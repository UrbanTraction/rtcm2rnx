#![feature(hash_extract_if)]

use std::{collections::HashMap,path::Path};

use clap::{value_parser, Arg, Command };
use rinex::{header::Header, observation::HeaderFields, prelude::{Constellation, Observable}, version::Version, Rinex};
use rtcmlib::RtcmDecoder;

// cli interface

fn command() -> clap::Command {
    Command::new("rtcm2rnx")
        .version("1.0")
        .author("Urban Traction, Inc.")
        .about("RTCM3 to RINEX OBS converter")
    
        .subcommand(
            Command::new("convert")
                .about("converts an input file")
                .arg(
                    Arg::new("use-rtklib-lli")
                        .long("use-rtklib-lli")
                        .help("Use the simplifed rtklib lli algo (for diagnostics only)")
                        .value_parser(value_parser!(bool))
                        .default_value("false"))
                .arg(
                    Arg::new("file_path")
                        .help("Log file input")
                        .required(true)
                        .index(1),
                )
            )
}


pub fn convert_file(file_path:&String, use_rtklib_lli:bool) {
    
    println!("converting rtcm file: {}", file_path);

    let mut rtcm_decoder = RtcmDecoder::new(use_rtklib_lli);

    let rtcm_file_path = Path::new(file_path);

    rtcm_decoder.load_file(rtcm_file_path);

    let mut observed_signals = rtcm_decoder.extract_observed_signals();

    // implement optional compressed rnx ?
    //let crinex:Option<Crinex> = Some(Crinex {version : Version {major: 3, minor: 0}, prog: "rtcm2rnx".to_string(), date: Epoch::now().unwrap()});  

    let scaling:HashMap<(Constellation, Observable), u16> = HashMap::new();

    let mut codes:HashMap<Constellation, Vec<Observable>> = HashMap::new();

    let mut gps_codes:Vec<String> = observed_signals.extract_if(|c|c.0 == Constellation::GPS).map(|c: (Constellation, String)|c.1).collect();
    gps_codes.sort();


    let mut gps_observables:Vec<Observable> = Vec::new();

    for code in gps_codes.iter() {
        gps_observables.push(Observable::PseudoRange(format!("C{}", code)));
        gps_observables.push(Observable::Phase(format!("L{}", code)));
        gps_observables.push(Observable::Doppler(format!("D{}", code)));
        gps_observables.push(Observable::SSI(format!("S{}", code)));
    }

    codes.insert(Constellation::GPS, gps_observables);

    let mut galileo_codes:Vec<String> = observed_signals.extract_if(|c|c.0 == Constellation::Galileo).map(|c: (Constellation, String)|c.1).collect();
    galileo_codes.sort();

    let mut galileo_observables:Vec<Observable> = Vec::new();

    for code in galileo_codes.iter() {

        galileo_observables.push(Observable::PseudoRange(format!("C{}", code)));
        galileo_observables.push(Observable::Phase(format!("L{}", code)));
        galileo_observables.push(Observable::Doppler(format!("D{}", code)));
        galileo_observables.push(Observable::SSI(format!("S{}", code)));

    }

    codes.insert(Constellation::Galileo, galileo_observables);
        
    let first_epoch = rtcm_decoder.get_first_epoch();
    let last_epoch = rtcm_decoder.get_last_epoch();

    let header_fields = HeaderFields {crinex : None, time_of_first_obs: first_epoch, time_of_last_obs: last_epoch, codes:codes, clock_offset_applied: false, scaling: scaling};

    let header : Header = Header::basic_obs();
    let header_obs = header.with_version(Version::new(3, 0)).with_observation_fields(header_fields);
    
    let record = rinex::record::Record::ObsRecord(rtcm_decoder.get_rtcm_data());
    let rinex = Rinex::new(header_obs, record);


    let rnx_path;
    
    if use_rtklib_lli {
        rnx_path = format!("{}.rtklib.rnx", file_path);
    }
    else {
        rnx_path = format!("{}.rnx", file_path);
    }
    
    rinex.to_file(&rnx_path).expect("unable to write file");

    println!("complete! RINEX file output: {}", rnx_path);

}


fn main() {
   
    let command = command();

    let matches = command.get_matches();

    match matches.subcommand() {

        Some(("convert", client_matches)) => {
            let file_path = client_matches.get_one::<String>("file_path").unwrap();
            let use_rtklib_lli= client_matches.get_one::<bool>("use-rtklib-lli").unwrap();
            convert_file(file_path, *use_rtklib_lli);
        }

        _ => {
            println!("Please use 'convert <rtcm file path>' command.");
            println!("Use --help for more information.");
        }
    }

}


