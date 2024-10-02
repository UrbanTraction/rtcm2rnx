use clap::{Arg, Command, };
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
            convert_file(file_path);
        }

        _ => {
            println!("Please use 'convert <rtcm file path>' command.");
            println!("Use --help for more information.");
        }
    }

}


