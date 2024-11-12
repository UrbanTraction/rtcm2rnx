use clap::{value_parser, Arg, Command };
use rtcmlib::convert_file;

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


