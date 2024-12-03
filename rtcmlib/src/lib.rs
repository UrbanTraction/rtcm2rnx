#![allow(warnings)] 

use std::{  borrow::{Borrow, BorrowMut}, collections::{BTreeMap, HashMap, HashSet}, fmt, fs::File, io::Read, path::Path, task::Context };
use hifitime::{Duration, Unit};
use log::info;
use rinex::{observation::{ Crinex, HeaderFields, LliFlags, ObservationData, EpochFlag}, prelude::{Constellation, Epoch, Header, Observable, SV}, version::Version, Rinex};

use rtcm_rs::{msg::{Msg1074T, Msg1077T, Msg1094T, Msg1097Data, Msg1097T, Msm46Sat, Msm57Sat}, Message, MsgFrameIter};


// epoch/sv/observation map for data extracted from rtcm log 
pub type RtcmData = BTreeMap<(Epoch, EpochFlag), (Option<f64>, BTreeMap<SV, HashMap<Observable, ObservationData>>)>;

pub mod prelude {
    pub use rinex::prelude::{Constellation, SV, Observable};
}


const SECONDS_PER_WEEK:u64 = 86400 * 7;

// const values transfered from rtklib.h and rtcm3.c
// https://github.com/rtklibexplorer/RTKLIB/blob/demo5/src/rtklib.h#L84

const C_LIGHT:f64 = 299792458.0;          // speed of light (m/s) 
const RANGE_MS:f64 = C_LIGHT * 0.001;     // range in 1ms 

const FREQL1:f64 = 1.57542e9;          /* L1/E1  frequency (Hz) */
const FREQL2:f64 = 1.22760e9;           /* L2     frequency (Hz) */
const FREQE5_B:f64 = 1.20714e9;           /* E5b    frequency (Hz) */
const FREQL5:f64 = 1.17645e9;           /* L5/E5a/B2a frequency (Hz) */
const FREQL6:f64 = 1.27875e9;           /* E6/L6 frequency (Hz) */
const FREQE5_AB:f64 = 1.191795e9;          /* E5a+b  frequency (Hz) */
const FREQ_S:f64 = 2.492028e9;           /* S      frequency (Hz) */
const FREQ1_GLO:f64 = 1.60200e9;           /* GLONASS G1 base frequency (Hz) */
const DFRQ1_GLO:f64 = 0.56250e6;           /* GLONASS G1 bias frequency (Hz/n) */
const FREQ2_GLO:f64 = 1.24600e9;           /* GLONASS G2 base frequency (Hz) */
const DFRQ2_GLO:f64 = 0.43750e6;           /* GLONASS G2 bias frequency (Hz/n) */
const FREQ3_GLO:f64 = 1.202025e9;          /* GLONASS G3 frequency (Hz) */
const FREQ1_A_GLO:f64 = 1.600995e9;          /* GLONASS G1a frequency (Hz) */
const FREQ2_A_GLO:f64 = 1.248060e9;          /* GLONASS G2a frequency (Hz) */
const FREQ1_CMP:f64 = 1.561098e9;          /* BDS B1I     frequency (Hz) */
const FREQ2_CMP:f64 = 1.20714e9;           /* BDS B2I/B2b frequency (Hz) */
const FREQ3_CM:f64 = 1.26852e9;          /* BDS B3      frequency (Hz) */

const DEFAULT_LLI:u16 = 0;

struct MsmData {

    constellation:Constellation, 
    satellite_id:u8, 
    band:u8, 
    attribute:char, 
    rough_range:Option<u8>, 
    rough_range_mod1ms:f64, 
    rough_phase_range_rate:Option<i16>, 
    fine_pseudo_range:Option<f64>, 
    fine_phase_range:Option<f64>, 
    fine_phase_range_rate:Option<f64>, 
    cnr:Option<f64>,
    loss_of_lock_indicator:u16,
    half_cycle_ambiguity:u8
}

pub struct LockStatus {
    use_rtklib_method:bool,
    previous_lli:HashMap<(SV, String), u16>,
    previous_epoch:HashMap<(SV, String), Epoch>
}

impl LockStatus {
    pub fn new(use_rtklib_method:bool) -> LockStatus {
        LockStatus { use_rtklib_method:use_rtklib_method, previous_lli: HashMap::new(), previous_epoch:HashMap::new()}
    }
    
    /// Calculates the minimum lock time (t) based on the indicator value (i).
    /// # Arguments
    /// * `i` - The indicator value from DF407.
    /// # Returns
    /// * The minimum lock time in milliseconds.
    fn calculate_minimum_lock_time(i: u16) -> u64 {
        let i = i as u64;
        match i {
            0..=63 => i,
            64..=95 => (2 * i - 64),
            96..=127 => (4 * i - 256),
            128..=159 => (8 * i - 768),
            160..=191 => (16 * i - 2048),
            192..=223 => (32 * i - 5120),
            224..=255 => (64 * i - 12288),
            256..=287 => (128 * i - 28672),
            288..=319 => (256 * i - 65536),
            320..=351 => (512 * i - 147456),
            352..=383 => (1024 * i - 327680),
            384..=415 => (2048 * i - 720896),
            416..=447 => (4096 * i - 1572864),
            448..=479 => (8192 * i - 3407872),
            480..=511 => (16384 * i - 7340032),
            512..=543 => (32768 * i - 15728640),
            544..=575 => (65536 * i - 33554432),
            576..=607 => (131072 * i - 71303168),
            608..=639 => (262144 * i - 150994944),
            640..=671 => (524288 * i - 318767104),
            672..=703 => (1048576 * i - 671088640),
            704 => (2097152 * i - 1409286144),
            _ => 0, // Reserved or out of range
        }
    }

    /// Returns the supplementary coefficient k based on the indicator value i.
    /// # Arguments
    /// * `i` - The indicator value from DF407.
    /// # Returns
    /// * The supplementary coefficient k.
    /// Returns the supplementary coefficient k based on the indicator value i.
    /// # Arguments
    /// * `i` - The indicator value from DF407.
    /// # Returns
    /// * The supplementary coefficient k.
    fn get_lli_coefficient(i: u16) -> u32 {
        let i = i as u32;
        match i {
            0..=63 => 1,
            64..=95 => 2,
            96..=127 => 4,
            128..=159 => 8,
            160..=191 => 16,
            192..=223 => 32,
            224..=255 => 64,
            256..=287 => 128,
            288..=319 => 256,
            320..=351 => 512,
            352..=383 => 1024,
            384..=415 => 2048,
            416..=447 => 4096,
            448..=479 => 8192,
            480..=511 => 16384,
            512..=543 => 32768,
            544..=575 => 65536,
            576..=607 => 131072,
            608..=639 => 262144,
            640..=671 => 524288,
            672..=703 => 1048576,
            704 => 2097152,
            _ => 0, // Reserved or out of range
        }
    }

    pub fn update_lock_status(&mut self, sv:&SV, code:&String, current_epoch:&Epoch, current_lli:u16, half_cycle_ambiguity:u8) -> Option<LliFlags> {
       
        let mut lli = LliFlags::OK_OR_UNKNOWN;

        let lock_key = &(*sv, code.clone());

        if half_cycle_ambiguity > 0 {
            // if half cycle slip possible 
            lli |= LliFlags::HALF_CYCLE_SLIP;
        }

        let previous_lli = *self.previous_lli.get(&lock_key).unwrap_or(&DEFAULT_LLI);
        
        if self.use_rtklib_method {

            // uses RTKLIB's simplified method for finding loss of lock
            // if current lli indicator is lower than the previous flag loss

            if (previous_lli == 0 && current_lli == 0) ||
                (current_lli < previous_lli) {
                    lli |= LliFlags::LOCK_LOSS;
            }
        }
        else {

            /// Determines if there is a loss of lock based on DF407 values and the calculated minimum lock times.
            /// According to RTCM 10403.4 section 3.5.12.3.2 Lock Time Indicator

            let previous_epoch = self.previous_epoch.get(lock_key);

            let mut dt:u64 = 0;

            if previous_epoch.is_some() {
                dt = (*current_epoch - *previous_epoch.unwrap()).to_unit(Unit::Millisecond) as u64;
            }

            let p = LockStatus::calculate_minimum_lock_time(previous_lli);
            let n = LockStatus::calculate_minimum_lock_time(current_lli);
            let a: u32 = LockStatus::get_lli_coefficient(previous_lli);
            let b = LockStatus::get_lli_coefficient(current_lli);

            if p > n {
                lli |= LliFlags::LOCK_LOSS;
            } else if p == n && dt >= a as u64 {
                lli |= LliFlags::LOCK_LOSS;
            } else if p == n && dt < a as u64 {
                lli |= LliFlags::OK_OR_UNKNOWN;
            } else if p < n && b > p as u32 && dt >= (n + b as u64 - p) {
                lli |= LliFlags::LOCK_LOSS;
            } else if p < n && b > p as u32 && n < dt && dt < (n + b as u64 - p) {
                lli |= LliFlags::LOCK_LOSS;
            } else if p < n && b > p as u32 && dt <= n {
                lli |= LliFlags::OK_OR_UNKNOWN;
            } else if p < n && b <= p as u32 && dt > n {
                lli |= LliFlags::LOCK_LOSS;
            } else if p < n && b <= p as u32 && dt <= n {
                lli |= LliFlags::OK_OR_UNKNOWN;
            } else {
                lli |= LliFlags::OK_OR_UNKNOWN;
            }
        }
        

        self.previous_lli.insert(lock_key.clone(), current_lli);
        self.previous_epoch.insert(lock_key.clone(), *current_epoch);

        return Some(lli);

    }
    
}




// constillation + code to frequency from RKTLIB:
// https://github.com/rtklibexplorer/RTKLIB/blob/demo5/src/rtkcmn.c#L733

fn get_frequency_gps(band:u8) -> f64 {

    match band {
        1 => { return FREQL1 }
        2 => { return FREQL2 }
        5 => { return FREQL5 }
        _ => { panic!("frequency not found"); }
    }
}

fn get_frequency_galileo(band:u8) -> f64 {

    match band {
        1 => { return FREQL1 }
        7 => { return FREQE5_B }
        5 => { return FREQL5 }
        6 => { return FREQL6 }
        8 => { return FREQE5_AB }
        _ => { panic!("frequency not found"); }
    }

}

fn get_frequency(constellation:Constellation, band:u8) -> f64 {
    match constellation {
        Constellation::GPS => {
            return get_frequency_gps(band);
        }
        Constellation::Galileo => {
            return get_frequency_galileo(band);
        }
        _ => {
            panic!("frequency not found");
        }
    }
}

// time conversion from GPS and Galileo time of week (ms) + GPS/Galileo week period
// see RTKLIB for implementation reference: https://github.com/tomojitakasu/RTKLIB/blob/71db0ffa0d9735697c6adfd06fdf766d0e5ce807/src/rtkcmn.c#L1246

pub fn rtcm_gps_time2epoch(tow_ms:f64, week:u64) -> Epoch {

    let mut tow_sec = tow_ms / 1000.0;

    if tow_sec < -1e9 || 1e9 < tow_sec {
        tow_sec = 0.0;
    }

    let t = Epoch::from_gpst_seconds(((week * SECONDS_PER_WEEK) as f64) + tow_sec);

    return t;
}

pub fn rtcm_galileo_time2epoch(tow_ms:f64, week:u64) -> Epoch {
    
    let mut tow_sec = tow_ms / 1000.0;

    if tow_sec < -1e9 || 1e9 < tow_sec {
        tow_sec = 0.0;
    }

    let t = Epoch::from_gst_seconds(((week * SECONDS_PER_WEEK) as f64) + tow_sec);

    return t;
}




pub struct RtcmDecoder {
    first_epoch:Option<Epoch>,
    last_epoch:Option<Epoch>,
    rtcm_data:RtcmData,
    lock_status:LockStatus
}


impl RtcmDecoder {

    pub fn new(use_rtklib_method:bool) -> Self {
        let rtcm_data = BTreeMap::new();
        let lock_status = LockStatus::new(use_rtklib_method);
        Self {first_epoch:None, last_epoch:None, rtcm_data, lock_status}
    }

    pub fn clear(&mut self) {
        self.first_epoch = None;
        self.last_epoch = None;
        self.rtcm_data = BTreeMap::new();
    }

    pub fn get_first_epoch(&self) -> Option<Epoch> {
        self.first_epoch.clone()
    }

    pub fn get_last_epoch(&self) -> Option<Epoch> {
        self.last_epoch.clone()
    }

    pub fn get_rtcm_data(&self) -> RtcmData {
        self.rtcm_data.clone()
    }

    fn process_signals(&mut self, signal:MsmData, msm_epoch:Epoch)  {
                            
        // modeled on RKTLIB msm7 decoder 
        // see: https://github.com/rtklibexplorer/RTKLIB/blob/demo5/src/rtcm3.c#L1987

        if self.first_epoch.is_none() || self.first_epoch.unwrap().gt(&msm_epoch) {
            self.first_epoch = Some(msm_epoch);
        }

        if self.last_epoch.is_none() || self.last_epoch.unwrap().lt(&msm_epoch) {
            self.last_epoch = Some(msm_epoch);
        }

        if !self.rtcm_data.contains_key(&(msm_epoch, EpochFlag::Ok)) {
            let mut epoch_group = (Some(0 as f64), BTreeMap::new());
            self.rtcm_data.insert((msm_epoch, EpochFlag::Ok), epoch_group);   
        }
        
        let epoch_data = self.rtcm_data.get_mut(&(msm_epoch, EpochFlag::Ok)).unwrap().1.borrow_mut();
        

        let code_str = format!("{}{}", signal.band, signal.attribute);

        let frequency:f64 = get_frequency(signal.constellation, signal.band);
        let wavelength:f64 = frequency / C_LIGHT;
        
        let sv_key = SV {constellation:signal.constellation, prn: signal.satellite_id};

        if !epoch_data.contains_key(&sv_key) {
            epoch_data.insert(sv_key, HashMap::new());
        }   
        
        let observation_data = epoch_data.get_mut(&sv_key).unwrap();   

        let mut range:Option<f64>  = None;
        if signal.rough_range.is_some() {
            range = Some(((signal.rough_range.unwrap() as f64) * RANGE_MS) + (signal.rough_range_mod1ms  * RANGE_MS));
        }

        let mut lli:Option<LliFlags> = self.lock_status.update_lock_status(&sv_key, &code_str, &msm_epoch, signal.loss_of_lock_indicator, signal.half_cycle_ambiguity);

    
        let mut rough_phase_range_rate:Option<f64> = None;
        if signal.rough_phase_range_rate.is_some()  {
            rough_phase_range_rate = Some(signal.rough_phase_range_rate.unwrap() as f64);
        } 
        
        let mut fine_phase_range_rate:Option<f64> = None;
        if signal.fine_phase_range_rate.is_some() {
            fine_phase_range_rate = signal.fine_phase_range_rate;
        }
        
        let mut fine_pseudo_range:Option<f64> = None;
        if signal.fine_pseudo_range.is_some() {
            fine_pseudo_range =  Some((signal.fine_pseudo_range.unwrap() as f64)  * RANGE_MS);
        }     

        let mut fine_pseudo_range:Option<f64> = None;
        if signal.fine_pseudo_range.is_some() {
            fine_pseudo_range = Some((signal.fine_pseudo_range.unwrap() as f64)  * RANGE_MS);
        }

        let mut fine_phase_range:Option<f64> = None;
        if signal.fine_phase_range.is_some() {
            fine_phase_range = Some((signal.fine_phase_range.unwrap() as f64)  * RANGE_MS);
        }
        
        
        if range.is_some() && fine_pseudo_range.is_some() {
            let pseudo_range_obs = (range.unwrap() + fine_pseudo_range.unwrap());
            let code = Observable::PseudoRange(format!("C{}", code_str));
            observation_data.insert(code, 
                                    ObservationData {obs:pseudo_range_obs, lli: None, snr: None});
        }
    
        if range.is_some() && fine_phase_range.is_some() {
            let phase_range_obs =(range.unwrap() + fine_phase_range.unwrap()) * wavelength; 
            let code = Observable::Phase(format!("L{}", code_str));
            observation_data.insert(code, 
                                    ObservationData {obs:phase_range_obs, lli: lli, snr: None});
        }
        
        if rough_phase_range_rate.is_some() && fine_phase_range_rate.is_some() {
            let phase_range_rate_obs:f64 = (-(rough_phase_range_rate.unwrap() + fine_phase_range_rate.unwrap())) * wavelength;
            let code = Observable::Doppler(format!("D{}", code_str));
            observation_data.insert(code, 
                                    ObservationData {obs:phase_range_rate_obs, lli: None, snr: None});
        }
        
        if signal.cnr.is_some() {
            let cnr_obs = (signal.cnr.unwrap() as f64);
            let code = Observable::SSI(format!("S{}", code_str));
            observation_data.insert(code, 
                                    ObservationData {obs:cnr_obs, lli: None, snr: None});
        }

    }

    pub fn process_msm1074(&mut self, msg:Msg1074T, msm_epoch:Epoch) {
                              
        let mut satellites:HashMap<u8,&Msm46Sat>  = HashMap::new();
            
        for satellite in msg.data_segment.satellite_data.iter() {
            satellites.insert(satellite.satellite_id, satellite);
        }

        let mut i = 0;
        for signal in msg.data_segment.signal_data.iter() {
        
            let cnr_u8:Option<u8> = signal.gnss_signal_cnr_dbhz;
            let mut cnr_f64:Option<f64> = None;

            if cnr_u8.is_some() {
                cnr_f64 = Some(cnr_u8.unwrap() as f64);
            } 

            let signal:MsmData  = MsmData {
                constellation: Constellation::GPS, 
                satellite_id: signal.satellite_id, 
                band: signal.signal_id.band(),
                attribute: signal.signal_id.attribute(),
                rough_range: satellites.get(&signal.satellite_id).unwrap().gnss_satellite_rough_range_integer_ms,
                rough_range_mod1ms: satellites.get(&signal.satellite_id).unwrap().gnss_satellite_rough_range_mod1ms_ms as f64,
                rough_phase_range_rate: None, 
                loss_of_lock_indicator: signal.gnss_phaserange_lock_time_ind as u16,
                half_cycle_ambiguity: signal.half_cycle_ambiguity_ind,
                fine_pseudo_range: signal.gnss_signal_fine_pseudorange_ms,
                fine_phase_range: signal.gnss_signal_fine_phaserange_ms,
                fine_phase_range_rate: None, 
                cnr: cnr_f64
            };

            self.process_signals(signal, msm_epoch);
    
        }
            
    }

    pub fn process_msm1077(&mut self, msg:Msg1077T, msm_epoch:Epoch) {
                                  
        let mut satellites:HashMap<u8,&Msm57Sat>  = HashMap::new();
            
        for satellite in msg.data_segment.satellite_data.iter() {
            satellites.insert(satellite.satellite_id, satellite);
        }

        for signal in msg.data_segment.signal_data.iter() {
            
            let signal:MsmData  = MsmData {
                constellation: Constellation::GPS, 
                satellite_id: signal.satellite_id, 
                band: signal.signal_id.band(),
                attribute: signal.signal_id.attribute(),
                rough_range: satellites.get(&signal.satellite_id).unwrap().gnss_satellite_rough_range_integer_ms,
                rough_range_mod1ms: satellites.get(&signal.satellite_id).unwrap().gnss_satellite_rough_range_mod1ms_ms as f64,
                rough_phase_range_rate: satellites.get(&signal.satellite_id).unwrap().gnss_satellite_rough_phaserange_rates_m_s,
                loss_of_lock_indicator: signal.gnss_phaserange_lock_time_ext_ind,
                half_cycle_ambiguity: signal.half_cycle_ambiguity_ind,
                fine_pseudo_range: signal.gnss_signal_fine_pseudorange_ext_ms,
                fine_phase_range: signal.gnss_signal_fine_phaserange_ext_ms,
                fine_phase_range_rate: signal.gnss_signal_fine_phaserange_rate_m_s,
                cnr: signal.gnss_signal_cnr_ext_dbhz,
            };

            self.process_signals(signal, msm_epoch);

        }
            
    }

    pub fn process_msm1094(&mut self, msg:Msg1094T,msm_epoch:Epoch) {
                                  
        let mut satellites:HashMap<u8,&Msm46Sat>  = HashMap::new();
            
        for satellite in msg.data_segment.satellite_data.iter() {
            satellites.insert(satellite.satellite_id, satellite);
        }

        for signal in msg.data_segment.signal_data.iter() {
        
            let cnr_u8:Option<u8> = signal.gnss_signal_cnr_dbhz;
            let mut cnr_f64:Option<f64> = None;

            if cnr_u8.is_some() {
                cnr_f64 = Some(cnr_u8.unwrap() as f64);
            } 

            let signal:MsmData  = MsmData {
                constellation: Constellation::GPS, 
                satellite_id: signal.satellite_id, 
                band: signal.signal_id.band(),
                attribute: signal.signal_id.attribute(),
                rough_range: satellites.get(&signal.satellite_id).unwrap().gnss_satellite_rough_range_integer_ms,
                rough_range_mod1ms: satellites.get(&signal.satellite_id).unwrap().gnss_satellite_rough_range_mod1ms_ms as f64,
                rough_phase_range_rate: None, 
                loss_of_lock_indicator: signal.gnss_phaserange_lock_time_ind as u16,
                half_cycle_ambiguity: signal.half_cycle_ambiguity_ind,
                fine_pseudo_range: signal.gnss_signal_fine_pseudorange_ms,
                fine_phase_range: signal.gnss_signal_fine_phaserange_ms,
                fine_phase_range_rate: None, 
                cnr: cnr_f64
            };

            self.process_signals(signal, msm_epoch);

        }
            
    }

    pub fn process_msm1097(&mut self, msg:Msg1097T, msm_epoch:Epoch) {
        
        let mut observations: BTreeMap<SV, HashMap<Observable, ObservationData>> = BTreeMap::new();
                                    
        let mut satellites:HashMap<u8,&Msm57Sat>  = HashMap::new();
            
        for satellite in msg.data_segment.satellite_data.iter() {
            satellites.insert(satellite.satellite_id, satellite);
        }

        for signal in msg.data_segment.signal_data.iter() {

            let signal:MsmData  = MsmData {
                constellation: Constellation::Galileo, 
                satellite_id: signal.satellite_id, 
                band: signal.signal_id.band(),
                attribute: signal.signal_id.attribute(),
                rough_range: satellites.get(&signal.satellite_id).unwrap().gnss_satellite_rough_range_integer_ms,
                rough_range_mod1ms: satellites.get(&signal.satellite_id).unwrap().gnss_satellite_rough_range_mod1ms_ms as f64,
                rough_phase_range_rate: satellites.get(&signal.satellite_id).unwrap().gnss_satellite_rough_phaserange_rates_m_s,
                loss_of_lock_indicator: signal.gnss_phaserange_lock_time_ext_ind,
                half_cycle_ambiguity: signal.half_cycle_ambiguity_ind,
                fine_pseudo_range: signal.gnss_signal_fine_pseudorange_ext_ms,
                fine_phase_range: signal.gnss_signal_fine_phaserange_ext_ms,
                fine_phase_range_rate: signal.gnss_signal_fine_phaserange_rate_m_s,
                cnr: signal.gnss_signal_cnr_ext_dbhz
                
            };

            self.process_signals(signal, msm_epoch);
            
        }
            
    }

    // convenience function for rinex library to build header table of observed signal codes by constellation (e.g. GPS: C1C, L5Q ... )
    pub fn extract_observed_signals(&self) -> HashSet<(Constellation, String)> {

        let mut observed_signals = HashSet::new();

        for epoch in self.rtcm_data.values() {
            for sv in epoch.1.keys() {
                let constellation = sv.constellation;
                let observations = epoch.1.get(&sv).unwrap();
                for observable in observations.keys() {
                    let code = observable.code().unwrap();
                    observed_signals.insert((constellation, code));
                }   
            }
        }

        observed_signals
    }

    pub fn load_file(&mut self, file_path:&Path) {

        info!("converting rtcm file: {}", file_path.to_str().unwrap());

        let mut rtcm_file = File::open(file_path).expect(format!("Unable to open file: {}", file_path.to_str().unwrap()).as_str());

        let mut rtcm_buffer = Vec::<u8>::new();

        if let Ok(_) = rtcm_file.read_to_end(&mut rtcm_buffer) {

            let mut iterator = MsgFrameIter::new(rtcm_buffer.as_slice());

            let mut gps_week:Option<u64>  = None;
            let mut galileo_week:Option<u64>  = None;

       

            for message_frame in &mut iterator {
                if message_frame.message_number().is_some() {
                
                    let msg_data = message_frame.get_message();
                    match msg_data {

                        // gps ephemeris 
                        Message::Msg1019(msg1019) => {
                            // TODO handle GPS week rollover correctly
                            gps_week = Some(msg1019.gps_week_number as u64 + 1024 + 1024);   
                            println!("gps week: {}", gps_week.unwrap());
                        }

                        // galileo i/nav ephemeris (need to check f/nav 1042 as well?)
                        Message::Msg1046(msg1046) => {
                            galileo_week = Some(msg1046.gal_week_number as u64);  
                            println!("galileo week: {}", galileo_week.unwrap());
                        }
                        
                        // gps msm7 
                        Message::Msg1074(msg1074) => {
                        
                            // wait for ephemeris gpst week before processing MSM7
                            if gps_week.is_some() {

                                let time = msg1074.gps_epoch_time_ms as f64;
                                let msm_epoch = rtcm_gps_time2epoch(time, gps_week.unwrap());
                                
                                self.process_msm1074(msg1074, msm_epoch);
                            }
                            
                        }      

                        // gps msm7 
                        Message::Msg1077(msg1077) => {
                        
                            // wait for ephemeris gpst week before processing MSM7
                            if gps_week.is_some() {

                                let time = msg1077.gps_epoch_time_ms as f64;
                                let msm_epoch = rtcm_gps_time2epoch(time, gps_week.unwrap());

                                self.process_msm1077(msg1077, msm_epoch);
                            }
                            
                        }      

                        // galileo msm7 
                        Message::Msg1094(msg1094) => {
                        
                            // wait for ephemeris gpst week before processing MSM7
                            if galileo_week.is_some() {
                                let time = msg1094.gal_epoch_time_ms as f64;
                                let msm_epoch = rtcm_galileo_time2epoch(time, galileo_week.unwrap());

                                self.process_msm1094(msg1094, msm_epoch);
                            }
                            
                        }         

                        // galileo msm7 
                        Message::Msg1097(msg1097) => {
                        
                            // wait for ephemeris gpst week before processing MSM7
                            if galileo_week.is_some() {
                                let time = msg1097.gal_epoch_time_ms as f64;
                                let msm_epoch = rtcm_galileo_time2epoch(time, galileo_week.unwrap());

                                self.process_msm1097(msg1097, msm_epoch);

                            }
                            
                        }         

                        _ => {
                            
                        }
                    }
                }   
            }
        }
    }
}


