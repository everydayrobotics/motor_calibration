mod cybergear;
use cybergear::CyberGearCanBusable;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut slcan = cybergear::SLCan::default().expect("couldn't open port");
    slcan.set_motor_param_bytes(127, 0x7005, [0x01, 0x00, 0x00, 0x00]);
    let output = slcan.enable_motor(127).expect("success");
    println!("${output}");
    Ok(())
}