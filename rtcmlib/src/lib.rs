#![allow(warnings)] 

use std::{  collections::{BTreeMap, HashMap}, fs::{File}, io::{Read} };
use rinex::{observation::{ HeaderFields, ObservationData}, prelude::{Constellation, Epoch, EpochFlag, Header, Observable, SV}, version::Version, Rinex};

use rtcm_rs::{msg::{Msg1077T, Msg1097Data, Msg1097T, Msm57Sat}, Message, MsgFrameIter};

pub mod prelude {
    pub use rinex::prelude::{Constellation, SV, Observable};
}


const SECONDS_PER_WEEK:u64 = 86400 * 7;

// const values transfered from rtklib.h and rtcm3.c
// https://github.com/rtklibexplorer/RTKLIB/blob/demo5/src/rtklib.h#L84

const C_LIGHT:f64 = 299792458.0;          // speed of light (m/s) 
const P2_10:f64 = 0.0009765625;          // 0.0009765625;          // 2^-10 
const P2_29:f64 = 1.862645149230957e-09; // 2^-29 
const P2_31:f64 = 4.656612873077393e-10; // 2^-31
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

struct Msm7Data {

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
    cnr:Option<f64>
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

fn rtcm_gps_time2epoch(tow_ms:f64, week:u64) -> Epoch {

    let mut tow_sec = tow_ms / 1000.0;

    if tow_sec < -1e9 || 1e9 < tow_sec {
        tow_sec = 0.0;
    }

    let t = Epoch::from_gpst_seconds(((week * SECONDS_PER_WEEK) as f64) + tow_sec);

    return t;
}

fn rtcm_galileo_time2epoch(tow_ms:f64, week:u64) -> Epoch {
    
    let mut tow_sec = tow_ms / 1000.0;

    if tow_sec < -1e9 || 1e9 < tow_sec {
        tow_sec = 0.0;
    }

    let t = Epoch::from_gst_seconds(((week * SECONDS_PER_WEEK) as f64) + tow_sec);

    return t;
}

fn process_signals( observations:&mut BTreeMap<SV, HashMap<Observable, ObservationData>>, 
                    signal:Msm7Data)  {
                        
    let code_str = format!("{}{}", signal.band, signal.attribute);

    let frequency:f64 = get_frequency(signal.constellation, signal.band);
    let wavelength:f64 = frequency / C_LIGHT;

    // modeled on RKTLIB msm7 decoder 
    // see: https://github.com/rtklibexplorer/RTKLIB/blob/demo5/src/rtcm3.c#L1987

    let observation_data:&mut HashMap<Observable, ObservationData>;

    let key = SV {constellation:signal.constellation, prn: signal.satellite_id};

    if observations.contains_key(&key) {
        observation_data = observations.get_mut(&key).unwrap();
    }   
    else {
        let observation_data_original = HashMap::new();
        observations.insert(key, observation_data_original);
        observation_data = observations.get_mut(&key).unwrap();                                                
    }

    let mut range:Option<f64>  = None;
    if signal.rough_range.is_some() {
        range = Some(((signal.rough_range.unwrap() as f64) * RANGE_MS) + (signal.rough_range_mod1ms  * RANGE_MS));
    }
   
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
                                ObservationData {obs:phase_range_obs, lli: None, snr: None});
    }
    
    if rough_phase_range_rate.is_some() && fine_phase_range_rate.is_some() {
        let phase_range_rate_obs:f64 = (-(rough_phase_range_rate.unwrap() + fine_phase_range_rate.unwrap())) * wavelength;
        let code = Observable::Doppler(format!("D{}", code_str));
        observation_data.insert(code, 
                                ObservationData {obs:phase_range_rate_obs, lli: None, snr: None});
    }
    
    if signal.cnr.is_some() {
        let cnr_obs = (signal.cnr.unwrap() as f64) / 0.001 + 0.5 ;
        let code = Observable::SSI(format!("S{}", code_str));
        observation_data.insert(code, 
                                ObservationData {obs:cnr_obs, lli: None, snr: None});
    }

}


pub fn process_msm1077( msg:Msg1077T)  -> BTreeMap<SV, HashMap<Observable, ObservationData>> {
    
    let mut observations: BTreeMap<SV, HashMap<Observable, ObservationData>> = BTreeMap::new();
                                
    let mut satellites:HashMap<u8,&Msm57Sat>  = HashMap::new();
        
    for satellite in msg.data_segment.satellite_data.iter() {
        satellites.insert(satellite.satellite_id, satellite);
    }

    let mut i = 0;
    for signal in msg.data_segment.signal_data.iter() {
    
        // if  signal.gnss_signal_fine_pseudorange_ext_ms.is_none() || 
        //     signal.gnss_signal_fine_phaserange_ext_ms.is_none() || 
        //     signal.gnss_signal_fine_phaserange_rate_m_s.is_none() ||
        //     signal.gnss_signal_cnr_ext_dbhz.is_none() {
        //     continue;
        // }

        let signal:Msm7Data  = Msm7Data {
            constellation: Constellation::GPS, 
            satellite_id: signal.satellite_id, 
            band: signal.signal_id.band(),
            attribute: signal.signal_id.attribute(),
            rough_range: satellites.get(&signal.satellite_id).unwrap().gnss_satellite_rough_range_integer_ms,
            rough_range_mod1ms: satellites.get(&signal.satellite_id).unwrap().gnss_satellite_rough_range_mod1ms_ms as f64,
            rough_phase_range_rate: satellites.get(&signal.satellite_id).unwrap().gnss_satellite_rough_phaserange_rates_m_s,
            fine_pseudo_range: signal.gnss_signal_fine_pseudorange_ext_ms,
            fine_phase_range: signal.gnss_signal_fine_phaserange_ext_ms,
            fine_phase_range_rate: signal.gnss_signal_fine_phaserange_rate_m_s,
            cnr: signal.gnss_signal_cnr_ext_dbhz,
        };

        process_signals(&mut observations, signal);

        i += 1;
        
        for a in observations.iter() {
            for b in a.1.iter() {
                println!("   {} {}: {}", a.0, b.0, b.1.obs);
            }
        }
    }

    return observations;
        
}

pub fn process_msm1097( msg:Msg1097T)  -> BTreeMap<SV, HashMap<Observable, ObservationData>> {
    
    let mut observations: BTreeMap<SV, HashMap<Observable, ObservationData>> = BTreeMap::new();
                                
    let mut satellites:HashMap<u8,&Msm57Sat>  = HashMap::new();
        
    for satellite in msg.data_segment.satellite_data.iter() {
        satellites.insert(satellite.satellite_id, satellite);
    }

    let mut i = 0;
    for signal in msg.data_segment.signal_data.iter() {
    
        // if  signal.gnss_signal_fine_pseudorange_ext_ms.is_none() || 
        //     signal.gnss_signal_fine_phaserange_ext_ms.is_none() || 
        //     signal.gnss_signal_fine_phaserange_rate_m_s.is_none() ||
        //     signal.gnss_signal_cnr_ext_dbhz.is_none() {
        //     continue;
        // }

        if signal.satellite_id == 11 {
            println!("debug")
        }

        let signal:Msm7Data  = Msm7Data {
            constellation: Constellation::Galileo, 
            satellite_id: signal.satellite_id, 
            band: signal.signal_id.band(),
            attribute: signal.signal_id.attribute(),
            rough_range: satellites.get(&signal.satellite_id).unwrap().gnss_satellite_rough_range_integer_ms,
            rough_range_mod1ms: satellites.get(&signal.satellite_id).unwrap().gnss_satellite_rough_range_mod1ms_ms as f64,
            rough_phase_range_rate: satellites.get(&signal.satellite_id).unwrap().gnss_satellite_rough_phaserange_rates_m_s,
            fine_pseudo_range: signal.gnss_signal_fine_pseudorange_ext_ms,
            fine_phase_range: signal.gnss_signal_fine_phaserange_ext_ms,
            fine_phase_range_rate: signal.gnss_signal_fine_phaserange_rate_m_s,
            cnr: signal.gnss_signal_cnr_ext_dbhz,
        };

        process_signals(&mut observations, signal);

        i += 1;
        
    }

    return observations;
        
}

pub fn convert_file(file_path:&String) {
    
    println!("converting rtcm file: {}", file_path);
    let mut rtcm_file = File::open(file_path).expect(format!("Unable to open file: {}", file_path).as_str());

    let mut rtcm_buffer = Vec::<u8>::new();

    if let Ok(_) = rtcm_file.read_to_end(&mut rtcm_buffer) {

        // implement optional compressed rnx ?
        //let crinex:Option<Crinex> = Some(Crinex {version : Version {major: 3, minor: 0}, prog: "rtcm2rnx".to_string(), date: Epoch::now().unwrap()});  

        let scaling:HashMap<(Constellation, Observable), u16> = HashMap::new();

        let mut rinex_data : BTreeMap<(Epoch, EpochFlag), (Option<f64>, BTreeMap<SV, HashMap<Observable, ObservationData>>)> = BTreeMap::new();
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
                        }

                        // galileo i/nav ephemeris (need to check f/nav 1042 as well?)
                        Message::Msg1046(msg1046) => {
                            galileo_week = Some(msg1046.gal_week_number as u64);  
                        }
                        
                        // gps msm7 
                        Message::Msg1077(msg1077) => {
                        
                            // wait for ephemeris gpst week before processing MSM7
                            if gps_week.is_some() {

                                let time = msg1077.gps_epoch_time_ms as f64;
                                let msm_epoch = rtcm_gps_time2epoch(time, gps_week.unwrap());

                            
                                let mut buff:[u8; 1200] = [0;1200];

                                let mut i = 0;
                                for u in message_frame.frame_data().iter() {
                                    buff[i] = *u;
                                    i += 1;
                                }

                                let mut observations = process_msm1077(msg1077);

                                if rinex_data.contains_key(&(msm_epoch, EpochFlag::Ok)) {
                                    rinex_data.get_mut(&(msm_epoch, EpochFlag::Ok)).unwrap().1.append(&mut observations);
                                
                                }
                                else {
                                    let mut observation_group = (Some(0 as f64), observations);
                                    rinex_data.insert((msm_epoch, EpochFlag::Ok), observation_group);
                                }
                            }
                            
                        }      

                        // galileo msm7 
                        Message::Msg1097(msg1097) => {
                        
                            // wait for ephemeris gpst week before processing MSM7
                            if galileo_week.is_some() {
                                let time = msg1097.gal_epoch_time_ms as f64;
                                let msm_epoch = rtcm_galileo_time2epoch(time, galileo_week.unwrap());

                            
                                let mut buff:[u8; 1200] = [0;1200];

                                let mut i = 0;
                                for u in message_frame.frame_data().iter() {
                                    buff[i] = *u;
                                    i += 1;
                                }

                                let mut observations = process_msm1097(msg1097);

                                if rinex_data.contains_key(&(msm_epoch, EpochFlag::Ok)) {
                                    rinex_data.get_mut(&(msm_epoch, EpochFlag::Ok)).unwrap().1.append(&mut observations);
                                
                                }
                                else {
                                    let mut observation_group = (Some(0 as f64), observations);
                                    rinex_data.insert((msm_epoch, EpochFlag::Ok), observation_group);
                                }
                            }
                            
                        }         

                        _ => {
                            
                        }
                    }
                }   
        }

        let mut codes:HashMap<Constellation, Vec<Observable>> = HashMap::new();

        codes.insert(Constellation::GPS, [Observable::PseudoRange("C1C".to_string()),
                                                Observable::Phase("L1C".to_string()),
                                                Observable::Doppler("D1C".to_string()),
                                                Observable::SSI("S1C".to_string()),
                                                Observable::PseudoRange("C5Q".to_string()),
                                                Observable::Phase("L5Q".to_string()),
                                                Observable::Doppler("D5Q".to_string()),
                                                Observable::SSI("S5Q".to_string())].to_vec());            

        codes.insert(Constellation::Galileo, [Observable::PseudoRange("C1C".to_string()),
                                                    Observable::Phase("L1C".to_string()),
                                                    Observable::Doppler("D1C".to_string()),
                                                    Observable::SSI("S1C".to_string()),
                                                    Observable::PseudoRange("C5Q".to_string()),
                                                    Observable::Phase("L5Q".to_string()),
                                                    Observable::Doppler("D5Q".to_string()),
                                                    Observable::SSI("S5Q".to_string())].to_vec());  
    
        let header_fields = HeaderFields {crinex : None, time_of_first_obs: None, time_of_last_obs: None, codes:codes, clock_offset_applied: false, scaling: scaling};

        let header : Header = Header::basic_obs();
        let header_obs = header.with_version(Version::new(3, 0)).with_observation_fields(header_fields);
        
        let record = rinex::record::Record::ObsRecord(rinex_data);
        let rinex = Rinex::new(header_obs, record);

        let rnx_path = format!("{}.rnx", file_path);
        rinex.to_file(&rnx_path).expect("unable to write file");

        println!("complete! RINEX file output: {}", rnx_path);
    }
}

