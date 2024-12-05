#![allow(warnings)] 

use std::borrow::Borrow;
use std::collections::{BTreeMap, HashMap};
use std::option::Iter;
use std::{fs::File, io::Read, mem::MaybeUninit, ptr::addr_of_mut};
use clap::builder::Str;
use cty::uint16_t;
use float_cmp::approx_eq;
use rinex::observation::LliFlags;
use rinex::prelude::EpochFlag;
use rtcm_rs::msg::Msg1127T;
use rtcm_rs::{msg, Message, MsgFrameIter};
use rtklib_sys::rtklib::{self, decode_msm7, obsd_t, rtcm_t};
use rinex::{observation::{ HeaderFields, ObservationData}};
use rtcmlib::{rtcm_galileo_time2epoch, rtcm_gps_time2epoch, LockStatus, RtcmDecoder};
use rtcmlib::prelude::{SV,Constellation, Observable};


const OBS_CODES:[&str;70]=[       /* observation code strings */

    ""  ,"1C","1P","1W","1Y", "1M","1N","1S","1L","1E", /*  0- 9 */
    "1A","1B","1X","1Z","2C", "2D","2S","2L","2X","2P", /* 10-19 */
    "2W","2Y","2M","2N","5I", "5Q","5X","7I","7Q","7X", /* 20-29 */
    "6A","6B","6C","6X","6Z", "6S","6L","8I","8Q","8X", /* 30-39 */
    "2I","2Q","6I","6Q","3I", "3Q","3X","1I","1Q","5A", /* 40-49 */
    "5B","5C","9A","9B","9C", "9X","1D","5D","5P","5Z", /* 50-59 */
    "6E","7D","7P","7Z","8D", "8P","4A","4B","4X",""    /* 60-69 */
];

fn compare(rtklib_obs:&obsd_t, rtcmlib_observations:&BTreeMap<SV, HashMap<Observable, ObservationData>>, sv:SV) {
    let rtcmlib_obs = rtcmlib_observations.get(&sv);

    let l = LliFlags::HALF_CYCLE_SLIP | LliFlags::LOCK_LOSS;
    if rtcmlib_obs.is_some() {
        // rtklib packs three channel observations into each osdb_t struct
        for i in [0,1,2] {
            let code = OBS_CODES[rtklib_obs.code[i] as usize];
            let rtcm_c_opt = rtcmlib_obs.unwrap().get(&Observable::PseudoRange(format!("C{}", code)));
            if !code.eq("") {
                if rtcm_c_opt.is_some()  {
                    let rtk_val = rtklib_obs.P[i];
                    let rtcm_val = rtcm_c_opt.unwrap().obs;
                    assert!(approx_eq!(f64, rtk_val, rtcm_val));
                }
                else if rtklib_obs.P[i] != 0.0 {
                    assert!(false, "missing rtcmlib observation");
                } 
            }
            

            let rtcm_l_opt = rtcmlib_obs.unwrap().get(&Observable::Phase(format!("L{}", code)));
            if !code.eq("") {
                if rtcm_l_opt.is_some() {
                    let rtk_val = rtklib_obs.L[i];
                    let rtcm_val = rtcm_l_opt.unwrap().obs;
                    assert!(approx_eq!(f64, rtk_val, rtcm_val));  


                    let rtk_lli_val = rtklib_obs.LLI[i];
                    let rtcm_lli_val = rtcm_l_opt.unwrap().lli;

                    match rtcm_lli_val {
                        None => {
                            assert_eq!(rtk_lli_val, 0);     
                        }
                        Some(LliFlags::OK_OR_UNKNOWN) => {
                            assert_eq!(rtk_lli_val, 0);     
                        }
                        Some(LliFlags::LOCK_LOSS) => {
                            assert_eq!(rtk_lli_val, 1);     
                        }
                        Some(LliFlags::HALF_CYCLE_SLIP) => {
                            assert_eq!(rtk_lli_val, 2);     
                        }
                        Some(l) => {
                            assert_eq!(rtk_lli_val, 3);     
                        }
                        _ => {
                            panic!("Unhandled LLI test case");
                        }
                    }                                       
                }
                else if rtklib_obs.L[i] != 0.0 {
                    assert!(false, "missing rtcmlib observation");
                }
            }

            // dopper calulated as v64 but returned from rtklib as f32 -- reduce precision for comparison 
            let rtcm_d_opt = rtcmlib_obs.unwrap().get(&Observable::Doppler(format!("D{}", code)));
            if !code.eq("") {
                if rtcm_d_opt.is_some()  {
                    let rtk_val = rtklib_obs.D[i] as f32;
                    let rtcm_val = rtcm_d_opt.unwrap().obs as f32;
                    assert!(approx_eq!(f32, rtk_val, rtcm_val));                                            
                }
                else if rtklib_obs.D[i] != 0.0 {
                    assert!(false, "missing rtcmlib observation");
                }
            }

            // snr rtklib output reduced to u16
            let rtcm_s_opt = rtcmlib_obs.unwrap().get(&Observable::SSI(format!("S{}", code)));
            if !code.eq("") {
                if rtcm_s_opt.is_some()  {
                    let rtk_val = rtklib_obs.SNR[i] as uint16_t;
                    // rtklib stores SNR float values as a uint16 shifted to 0.001 precision 
                    let rtcm_val = (rtcm_s_opt.unwrap().obs / 0.001 + 0.5) as uint16_t;
                    assert_eq!(rtk_val, rtcm_val);                                            
                }
                else if rtklib_obs.SNR[i] != 0{
                    assert!(false, "missing rtcmlib observation");
                }
            }
        }
       
    }
}

#[test]
fn process_rtcm() {
    // let ref mut cool = CoolStruct {x: 0, y: 0};
    let file_path = "tests/data/debug_bds.rtcm";
    // unsafe { cool_function(2, 6, cool) }
    let mut rtcm_file = File::open(file_path).expect(format!("Unable to open file: {}", file_path).as_str());

    let mut rtcm_buffer = Vec::<u8>::new();

    //let mut lock_status:LockStatus = LockStatus::new(false);
    let mut rtcm_decoder= RtcmDecoder::new(true);

    if let Ok(_) = rtcm_file.read_to_end(&mut rtcm_buffer) {

        let mut iterator = MsgFrameIter::new(rtcm_buffer.as_slice());

        let mut gps_week:Option<u64>  = Some(2339);
        let mut galileo_week:Option<u64>  = Some(1315);
        let mut bds_week:Option<u64>  = Some(983);

        for message_frame in &mut iterator {
            match message_frame.get_message() {
                
                  // gps ephemeris 
                Message::Msg1019(msg1019) => {
                    // TODO handle GPS week rollover correctly
                    gps_week = Some(msg1019.gps_week_number as u64 + 1024 + 1024);   
                    println!("gps week: {}", gps_week.unwrap());
                }

                // galileo i/nav ephemeris (need to check f/nav 1042 as well?)
                Message::Msg1042(msg1042) => {
                    bds_week = Some(msg1042.bds_week_number as u64);  
                    println!("bds week: {}", bds_week.unwrap());
                }

                // galileo i/nav ephemeris (need to check f/nav 1042 as well?)
                Message::Msg1046(msg1046) => {
                    galileo_week = Some(msg1046.gal_week_number as u64);  
                    println!("galileo week: {}", galileo_week.unwrap());
                }

                // gps 
                Message::Msg1077(msg1077) => {

                    let time = msg1077.gps_epoch_time_ms as f64;
                    let msm_epoch = rtcm_gps_time2epoch(time, gps_week.unwrap());

                    unsafe { 
                        
                        let mut rtcm:MaybeUninit<rtcm_t> = MaybeUninit::zeroed();
                        let rtcm_ptr = rtcm.as_mut_ptr();

                        
                        let mut buff:[u8;1200] = [0;1200]; 
                        let mut i = 0;
                        for b in message_frame.frame_data() {
                            buff[i] = *b;
                            i += 1;
                        }

                        let mut rtklib_observations:MaybeUninit<[obsd_t;24]>= MaybeUninit::zeroed(); 
                        let rtklib_observations_ptr = rtklib_observations.assume_init_mut().as_mut_ptr();

                        addr_of_mut!((*rtcm_ptr).obs.data).write(rtklib_observations_ptr);
                        addr_of_mut!((*rtcm_ptr).buff).write(buff);
                        addr_of_mut!((*rtcm_ptr).len).write( message_frame.frame_len() as i32);
                        
                        // calc rtklib values
                        decode_msm7(rtcm.as_mut_ptr(), 0x01);

                        // calc rtcmlib values
                        let rtcmlib_observations = rtcm_decoder.process_msm1077(msg1077, msm_epoch);

                        let mut obs_stats = 0;
                        let rtk = rtklib_observations.assume_init();
                        for rtklib_obs in rtk {
                            if rtklib_obs.sat > 0 {
                                obs_stats += 1;
                                // rtklib gps sat no aligns with prn -- sat no for other constellations aren't the same as prn 
                                let sv = SV {prn: rtklib_obs.sat , constellation:Constellation::GPS};

                                let rtcm_data = rtcm_decoder.get_rtcm_data();
                                let epoch_data = rtcm_data.get(&(msm_epoch, EpochFlag::Ok)).unwrap().1.borrow();
                                compare(&rtklib_obs, epoch_data, sv);
                            }
                        }   
                    }
                }

                // galileo  
                Message::Msg1097(msg1097) => {
                    
                    let time = msg1097.gal_epoch_time_ms as f64;
                    let msm_epoch = rtcm_galileo_time2epoch(time, galileo_week.unwrap());

                    unsafe { 
                        let mut rtcm:MaybeUninit<rtcm_t> = MaybeUninit::zeroed();
                        let rtcm_ptr = rtcm.as_mut_ptr();

                        
                        let mut buff:[u8;1200] = [0;1200]; 
                        let mut i = 0;
                        for b in message_frame.frame_data() {
                            buff[i] = *b;
                            i += 1;
                        }
                    
                        let mut rtklib_observations:MaybeUninit<[obsd_t;24]>= MaybeUninit::zeroed(); 
                        let rtklib_observations_ptr = rtklib_observations.assume_init_mut().as_mut_ptr();

                        addr_of_mut!((*rtcm_ptr).obs.data).write(rtklib_observations_ptr);
                        addr_of_mut!((*rtcm_ptr).buff).write(buff);
                        addr_of_mut!((*rtcm_ptr).len).write( message_frame.frame_len() as i32);
                        
                        // calc rtklib values
                        decode_msm7(rtcm.as_mut_ptr(), 0x08);

                        // calc rtcmlib values
                        let rtcmlib_observations = rtcm_decoder.process_msm1097(msg1097, msm_epoch);

                        let mut obs_stats = 0;
                        for rtklib_obs in rtklib_observations.assume_init() {
                            if rtklib_obs.sat > 0 {

                                // rtklib doesn't store prn internally -- need to decode their internal multi-constellation id
                                // rtk_no = NSATGPS+NSATGLO+prn-MINPRNGAL+1 
                                // this number will change if other constellations are enabled and can't be pulled from FFI as it's all #define
                                let galileo_sat_no = rtklib_obs.sat - 32;

                                obs_stats += 1;
                                let sv = SV {prn: galileo_sat_no , constellation:Constellation::Galileo};

                                let rtcm_data = rtcm_decoder.get_rtcm_data();
                                let epoch_data = rtcm_data.get(&(msm_epoch, EpochFlag::Ok)).unwrap().1.borrow();
                                compare(&rtklib_obs, epoch_data, sv);
                            }
                        }   
                    }
                }

                // bds
                Message::Msg1127(msg1127) => {
                        
                    let time = msg1127.bds_epoch_time_ms as f64;
                    let msm_epoch = rtcm_galileo_time2epoch(time, bds_week.unwrap());

                    // todo add  rtklib test support for bds msm7
                    //
                    // unsafe { 
                    //     let mut rtcm:MaybeUninit<rtcm_t> = MaybeUninit::zeroed();
                    //     let rtcm_ptr = rtcm.as_mut_ptr();

                        
                    //     let mut buff:[u8;1200] = [0;1200]; 
                    //     let mut i = 0;
                    //     for b in message_frame.frame_data() {
                    //         buff[i] = *b;
                    //         i += 1;
                    //     }
                    
                    //     let mut rtklib_observations:MaybeUninit<[obsd_t;24]>= MaybeUninit::zeroed(); 
                    //     let rtklib_observations_ptr = rtklib_observations.assume_init_mut().as_mut_ptr();

                    //     addr_of_mut!((*rtcm_ptr).obs.data).write(rtklib_observations_ptr);
                    //     addr_of_mut!((*rtcm_ptr).buff).write(buff);
                    //     addr_of_mut!((*rtcm_ptr).len).write( message_frame.frame_len() as i32);
                        
                    //     // calc rtklib values
                    //     decode_msm7(rtcm.as_mut_ptr(), 0x08);

                    //     // calc rtcmlib values
                    //     let rtcmlib_observations = rtcm_decoder.process_msm1127(msg1127, msm_epoch);

                    //     let mut obs_stats = 0;
                    //     for rtklib_obs in rtklib_observations.assume_init() {
                    //         if rtklib_obs.sat > 0 {

                    //             // rtklib doesn't store prn internally -- need to decode their internal multi-constellation id
                    //             // rtk_no = NSATGPS+NSATGLO+prn-MINPRNGAL+1 
                    //             // this number will change if other constellations are enabled and can't be pulled from FFI as it's all #define
                    //             let galileo_sat_no = rtklib_obs.sat - 32;

                    //             obs_stats += 1;
                    //             let sv = SV {prn: galileo_sat_no , constellation:Constellation::Galileo};

                    //             let rtcm_data = rtcm_decoder.get_rtcm_data();
                    //             let epoch_data = rtcm_data.get(&(msm_epoch, EpochFlag::Ok)).unwrap().1.borrow();
                    //             compare(&rtklib_obs, epoch_data, sv);
                    //         }
                    //     }   
                    // }
                }

                _ => {
                    println!("{}", message_frame.message_number().unwrap());
                }
            }            

        }
            
    }
}
    
    
