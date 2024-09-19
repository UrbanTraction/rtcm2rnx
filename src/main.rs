use std::{  borrow::BorrowMut, cmp::Ordering, collections::{BTreeMap, HashMap, HashSet}, fs::File, io::Read, ops::{Add, Div, Mul}};

use rinex::{antex::frequency, observation::{Crinex, HeaderFields, ObservationData}, prelude::{Constellation, Epoch, EpochFlag, Header, Observable, TimeScale, SV}, version::Version, Rinex};

use clap::{Arg, Command, };

use rtcm_rs::{next_msg_frame, Message, MessageFrame, MsgFrameIter};

const SECONDS_PER_WEEK:u64 = 86400 * 7;

// const values transfered from rtklib.h and rtcm3.c
// https://github.com/tomojitakasu/RTKLIB/blob/71db0ffa0d9735697c6adfd06fdf766d0e5ce807/src/rtklib.h#L59
// https://github.com/tomojitakasu/RTKLIB/blob/master/src/rtcm3.c#L40
const CLIGHT:f64 = 299792458.0;          // speed of light (m/s) 
const P2_10:f64 = 0.0009765625;          // 2^-10 
const P2_29:f64 = 1.862645149230957e-09; // 2^-29 
const P2_31:f64 = 4.656612873077393e-10; // 2^-31
const RANGE_MS:f64 = (CLIGHT*0.001);     // range in 1ms 

const FREQ1:f64  = 1.57542e9;  // L1/E1  frequency (Hz)
const FREQ2:f64  = 1.22760e9;  // L2     frequency (Hz) 
const FREQ5:f64  = 1.17645e9;  // L5/E5a frequency (Hz)
const FREQ6:f64  = 1.27875e9;  //  E6/LEX frequency (Hz) 
const FREQ7:f64  = 1.20714e9;  //  E5b    frequency (Hz) 
const FREQ8:f64  = 1.191795e9; // E5a+b  frequency (Hz)

// msm signal ids from RTKLIB

// const msm_sig_gps:[&str;32]=[
//     // GPS: ref [13] table 3.5-87, ref [14][15] table 3.5-91 
//     ""  ,"1C","1P","1W","1Y","1M",""  ,"2C","2P","2W","2Y","2M", /*  1-12 */
//     ""  ,""  ,"2S","2L","2X",""  ,""  ,""  ,""  ,"5I","5Q","5X", /* 13-24 */
//     ""  ,""  ,""  ,""  ,""  ,"1S","1L","1X"                      /* 25-32 */
// ];

// const msm_sig_gal:[&str;32]=[
//     // Galileo: ref [15] table 3.5-100 
//     ""  ,"1C","1A","1B","1X","1Z",""  ,"6C","6A","6B","6X","6Z",
//     ""  ,"7I","7Q","7X",""  ,"8I","8Q","8X",""  ,"5I","5Q","5X",
//     ""  ,""  ,""  ,""  ,""  ,""  ,""  ,""
// ];

const OBS_CODES:[&str;50] =[       /* observation code strings */
    
    ""  ,"1C","1P","1W","1Y", "1M","1N","1S","1L","1E", /*  0- 9 */
    "1A","1B","1X","1Z","2C", "2D","2S","2L","2X","2P", /* 10-19 */
    "2W","2Y","2M","2N","5I", "5Q","5X","7I","7Q","7X", /* 20-29 */
    "6A","6B","6C","6X","6Z", "6S","6L","8L","8Q","8X", /* 30-39 */
    "2I","2Q","6I","6Q","3I", "3Q","3X","1I","1Q",""    /* 40-49 */
];
const OBS_FREQUENCIES:[u8;50] = [ /* 1:L1,2:L2,3:L5,4:L6,5:L7,6:L8,7:L3 */
    
    0, 1, 1, 1, 1,  1, 1, 1, 1, 1, /*  0- 9 */
    1, 1, 1, 1, 2,  2, 2, 2, 2, 2, /* 10-19 */
    2, 2, 2, 2, 3,  3, 3, 5, 5, 5, /* 20-29 */
    4, 4, 4, 4, 4,  4, 4, 6, 6, 6, /* 30-39 */
    2, 2, 4, 4, 3,  3, 3, 1, 1, 0  /* 40-49 */
];


// satellite wavelength calc per RTKLIB
// https://github.com/tomojitakasu/RTKLIB/blob/71db0ffa0d9735697c6adfd06fdf766d0e5ce807/src/rtkcmn.c#L3162

fn get_wavelength(constellation:Constellation, freq:u8) -> f64 {

    if constellation == Constellation::BeiDou || constellation == Constellation::Glonass {
        panic!("constellation not implemented")
    }
    else  {
        if      (freq==0) { return CLIGHT / FREQ1; } /* L1/E1 */
        else if (freq==1) { return CLIGHT / FREQ2; } /* L2 */
        else if (freq==2) { return CLIGHT / FREQ5; } /* L5/E5a */
        else if (freq==3) { return CLIGHT / FREQ6; } /* L6/LEX */
        else if (freq==4) { return CLIGHT / FREQ7; } /* E5b */
        else if (freq==5) { return CLIGHT / FREQ8; } /* E5a+b */
        else {
            panic!("frequency not found")
        } 
    }

}

// perform look up in table based on RTKLIB OBS code->frequency mapping
// https://github.com/tomojitakasu/RTKLIB/blob/71db0ffa0d9735697c6adfd06fdf766d0e5ce807/src/rtkcmn.c#L525
fn get_frequency(obs_code:&String) -> u8 {

    let mut i = 0;
    for obs in OBS_CODES {
        if obs_code.eq(&obs.to_string()) {
            return OBS_FREQUENCIES[i];
        }
        i += 1;
    }
    panic!("frequency code not found: {}", obs_code);
}


// time conversion from GPS and Galileo time of week (ms) + GPS/Galileo week period
// see RTKLIB for implementation reference: https://github.com/tomojitakasu/RTKLIB/blob/71db0ffa0d9735697c6adfd06fdf766d0e5ce807/src/rtkcmn.c#L1246

fn rtcm_gps_time2epoch(tow_ms:f64, week:u64) -> Epoch {

    let mut tow_sec = tow_ms.div(1000.0);

    if tow_sec < -1e9 || 1e9 < tow_sec {
        tow_sec = 0.0;
    }

    let t = Epoch::from_gpst_seconds(((week * SECONDS_PER_WEEK) as f64) + tow_sec);

    return t;
}

fn rtcm_galileo_time2epoch(tow_ms:f64, week:u64) -> Epoch {
    
    let mut tow_sec = tow_ms.div(1000.0);

    if tow_sec < -1e9 || 1e9 < tow_sec {
        tow_sec = 0.0;
    }

    let t = Epoch::from_gst_seconds(((week * SECONDS_PER_WEEK) as f64) + tow_sec);

    return t;
}

fn calc_observation(r_in:f64, rr_in:f64, pr_in:f64, cp_in:f64, rrf_in:f64, cnr_in:f64, wavelength:f64, code_str:String ) -> HashMap<Observable, ObservationData> {
    let mut observation_data: HashMap<Observable, ObservationData> = HashMap::new();

    let r  = r_in * RANGE_MS * P2_10;
    let rr = rr_in;
    let pr = pr_in *  P2_29 * RANGE_MS;
    let cp = cp_in * P2_31 * RANGE_MS;
    let rrf = rrf_in * 0.0001;
    let cnr = cnr_in * 0.0625;
    
    let code = Observable::PseudoRange(format!("C{}", code_str));
    observation_data.insert(code, 
                            ObservationData {obs:(r+pr), lli: None, snr: None});
    
    let code = Observable::Phase(format!("L{}", code_str));
    observation_data.insert(code, 
                            ObservationData {obs:(r+cp)/wavelength, lli: None, snr: None});

    let code = Observable::Doppler(format!("D{}", code_str));
    observation_data.insert(code, 
                            ObservationData {obs:(-(rr-rrf)/wavelength), lli: None, snr: None});
    
    let code = Observable::SSI(format!("S{}", code_str));
    observation_data.insert(code, 
                            ObservationData {obs:cnr*4.0, lli: None, snr: None});

    return observation_data;

}

fn command() -> clap::Command {
    Command::new("rtcm2rnx")
        .version("1.0")
        .author("Urban Traction, Inc.")
        .about("RTCM3 to RINEX OBS converter")
    
        .subcommand(
            Command::new("convert")
                .about("converts an input file")
                .arg(
                    Arg::new("file_path")
                        .help("Log file input")
                        .required(true)
                        .index(1),
                )
            )
}

fn main() {
   
    let command = command();

    let matches = command.get_matches();

    match matches.subcommand() {

        Some(("convert", client_matches)) => {

            let file_path = client_matches.get_one::<String>("file_path").unwrap();

            println!("converting rtcm file: {}", file_path);
            let mut rtcm_file = File::open(file_path).expect(format!("Unable to open file: {}", file_path).as_str());

            let mut rtcm_buffer = Vec::<u8>::new();

            if let Ok(_) = rtcm_file.read_to_end(&mut rtcm_buffer) {

                let crinex:Option<Crinex> = Some(Crinex {version : Version {major: 3, minor: 0}, prog: "rtcm2rnx".to_string(), date: Epoch::now().unwrap()});           


                let scaling:HashMap<(Constellation, Observable), u16> = HashMap::new();
                

               

                let mut rinex_data : BTreeMap<(Epoch, EpochFlag), (Option<f64>, BTreeMap<SV, HashMap<Observable, ObservationData>>)> = BTreeMap::new();
                
                let mut iterator = MsgFrameIter::new(rtcm_buffer.as_slice());

                let mut gps_week:Option<u64>  = None;
                let mut galileo_week:Option<u64>  = None;

                let mut gps_codes: HashSet<Observable> = HashSet::new();
                let mut galileo_codes: HashSet<Observable> = HashSet::new();
                
                for message_frame in &mut iterator {
                    if message_frame.message_number().is_some() {
                            let msg_data = message_frame.get_message();

                            match msg_data {

                                // gps ephemeris 
                                Message::Msg1019(msg1019) => {

                                    // TODO handle GPS week rollover correctly
                                    gps_week = Some(msg1019.gps_week_number as u64 + 1024 + 1024);                 
                                    
                                }

                                // galileo i/nav ephemeris (need to check f/nav 1046 as well?)
                                Message::Msg1046(msg1046) => {
                                    galileo_week = Some(msg1046.gal_week_number as u64);   
                                }
                              
                                // gps msm7 
                                Message::Msg1077(msg1077) => {
                                
                                    // wait for ephemeris gpst week before processing MSM7
                                    if gps_week.is_some() {

                                        let time = msg1077.gps_epoch_time_ms as f64;
                                        let msm_epoch = rtcm_gps_time2epoch(time, gps_week.unwrap());

                                        let mut observations: BTreeMap<SV, HashMap<Observable, ObservationData>> = BTreeMap::new();
                                        
                                        let mut i = 0;
                                        for signal in msg1077.data_segment.signal_data.iter() {
                                            
                                            if msg1077.data_segment.satellite_data.len() <= i ||
                                                msg1077.data_segment.satellite_data[i].gnss_satellite_rough_range_integer_ms.is_none() ||
                                                msg1077.data_segment.satellite_data[i].gnss_satellite_rough_phaserange_rates_m_s.is_none() ||
                                                signal.gnss_signal_fine_pseudorange_ext_ms.is_none() || 
                                                signal.gnss_signal_fine_phaserange_ext_ms.is_none() || 
                                                signal.gnss_signal_fine_phaserange_rate_m_s.is_none() ||
                                                signal.gnss_signal_cnr_ext_dbhz.is_none() {
                                                continue;
                                            }

                                            let code_str = format!("{}{}", signal.signal_id.band(), signal.signal_id.attribute());

                                            let frequency = get_frequency(&code_str);
                                            let wavelength = get_wavelength(Constellation::GPS, frequency);

                                            // modeled on RKTLIB msm7 decoder 
                                            // see: https://github.com/tomojitakasu/RTKLIB/blob/71db0ffa0d9735697c6adfd06fdf766d0e5ce807/src/rtcm3.c#L2003

                                            let observation_data= calc_observation(
                                                                msg1077.data_segment.satellite_data[i].gnss_satellite_rough_range_integer_ms.unwrap() as f64, 
                                                                msg1077.data_segment.satellite_data[i].gnss_satellite_rough_phaserange_rates_m_s.unwrap() as f64,
                                                                signal.gnss_signal_fine_pseudorange_ext_ms.unwrap(),
                                                                signal.gnss_signal_fine_phaserange_ext_ms.unwrap(),
                                                                signal.gnss_signal_fine_phaserange_rate_m_s.unwrap(),
                                                                signal.gnss_signal_cnr_ext_dbhz.unwrap(),
                                                                wavelength,
                                                                code_str ); 

                                            for v in observation_data.keys().into_iter() {
                                                gps_codes.insert(v.clone());
                                            }          
                                            i += 1;
                                            
                                            observations.insert(SV {constellation:Constellation::GPS, prn: signal.satellite_id}, observation_data);
                                        }

                                        if rinex_data.get(&(msm_epoch, EpochFlag::Ok)).is_some() {

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
                                    
                                    // wait for galileo ephemeris gst week epoch
                                    if galileo_week.is_some() {
                                        let time = msg1097.gal_epoch_time_ms as f64;
                                        let msm_epoch = rtcm_galileo_time2epoch(time, galileo_week.unwrap());

                                        let mut observations: BTreeMap<SV, HashMap<Observable, ObservationData>> = BTreeMap::new();
                                        
                                        let mut i = 0;
                                        for signal in msg1097.data_segment.signal_data.iter() {
                                            
                                            if msg1097.data_segment.satellite_data.len() <= i ||
                                                msg1097.data_segment.satellite_data[i].gnss_satellite_rough_range_integer_ms.is_none() ||
                                                msg1097.data_segment.satellite_data[i].gnss_satellite_rough_phaserange_rates_m_s.is_none() ||
                                                signal.gnss_signal_fine_pseudorange_ext_ms.is_none() || 
                                                signal.gnss_signal_fine_phaserange_ext_ms.is_none() || 
                                                signal.gnss_signal_fine_phaserange_rate_m_s.is_none() ||
                                                signal.gnss_signal_cnr_ext_dbhz.is_none() {
                                                continue;
                                            }

                                            let code_str = format!("{}{}", signal.signal_id.band(), signal.signal_id.attribute());

                                            let frequency = get_frequency(&code_str);
                                            let wavelength = get_wavelength(Constellation::Galileo, frequency);

                                            // modeled on RKTLIB msm7 decoder 
                                            // see: https://github.com/tomojitakasu/RTKLIB/blob/71db0ffa0d9735697c6adfd06fdf766d0e5ce807/src/rtcm3.c#L2003

                                            let observation_data= calc_observation(
                                                                msg1097.data_segment.satellite_data[i].gnss_satellite_rough_range_integer_ms.unwrap() as f64, 
                                                                msg1097.data_segment.satellite_data[i].gnss_satellite_rough_phaserange_rates_m_s.unwrap() as f64,
                                                                signal.gnss_signal_fine_pseudorange_ext_ms.unwrap(),
                                                                signal.gnss_signal_fine_phaserange_ext_ms.unwrap(),
                                                                signal.gnss_signal_fine_phaserange_rate_m_s.unwrap(),
                                                                signal.gnss_signal_cnr_ext_dbhz.unwrap(),
                                                                wavelength,
                                                                code_str ); 

                                            for v in observation_data.keys().into_iter() {
                                                galileo_codes.insert(v.clone());
                                            }           
                                            i += 1;
                                            
                                            observations.insert(SV {constellation:Constellation::Galileo, prn: signal.satellite_id}, observation_data);
                                        }
                                      

                                        if rinex_data.get(&(msm_epoch, EpochFlag::Ok)).is_some() {

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

                // sort code order in RINEX header
                let mut gps_code_vec:Vec<Observable> = gps_codes.into_iter().collect();

                gps_code_vec.sort_by(|a,b | 
                    format!("{}{}", a.to_string().as_bytes()[1], a.to_string().as_bytes()[0]).cmp(&format!("{}{}", b.to_string().as_bytes()[1], b.to_string().as_bytes()[0]))
                 );
                codes.insert(Constellation::GPS, gps_code_vec);            

                let mut galileo_code_vec:Vec<Observable> = galileo_codes.into_iter().collect();
                galileo_code_vec.sort_by(|a,b | 
                    format!("{}{}", a.to_string().as_bytes()[1], a.to_string().as_bytes()[0]).cmp(&format!("{}{}", b.to_string().as_bytes()[1], b.to_string().as_bytes()[0]))
                );
                codes.insert(Constellation::Galileo, galileo_code_vec);  
            
                let header_fields = HeaderFields {crinex : None, time_of_first_obs: None, time_of_last_obs: None, codes:codes, clock_offset_applied: false, scaling: scaling};

                let header : Header = Header::basic_obs();
                let header_obs = header.with_observation_fields(header_fields);

                let record = rinex::record::Record::ObsRecord(rinex_data);
                let rinex = Rinex::new(header_obs, record);

                let rnx_path = format!("{}.rnx", file_path);
                rinex.to_file(&rnx_path).expect("unable to write file");

                println!("complete! RINEX file output: {}", rnx_path);
            }
        }

        _ => {
            println!("Please use 'convert <rtcm file path>' command.");
            println!("Use --help for more information.");
        }
    }

}
