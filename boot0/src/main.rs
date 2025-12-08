use core::time;
use std::thread::sleep;

use probe_rs::{MemoryInterface, Permissions, Session};
use probe_rs::probe::{list::Lister, Probe, WireProtocol, DebugProbeInfo};
use anyhow::{Result, Context};

fn main() -> Result<()> {
    // 1. Get a list of all available debug probes using the Lister.
    let lister = Lister::new();
    let mut probes = lister.list_all();
    
    // Check if any probes were found.
    if probes.is_empty() {
        anyhow::bail!("No debug probes found.");
    }

  
    set_option_bytes(&mut probes, 0)?;
    set_option_bytes(&mut probes, 1)?;
    
    Ok(())
}

fn set_option_bytes(probes: &mut Vec<DebugProbeInfo>, probe_index: usize) -> Result<()> 
{
    // Select and open the probe.
    let probe_info = &probes[probe_index];
    println!("Opening probe[{}]: {}", probe_index, probe_info.identifier);
    let mut probe: Probe = probe_info.open()
        .context("Failed to open the selected probe")?;

    // Optional: Select the wire protocol (SWD is default for most ARM Cortex-M)
    probe.select_protocol(WireProtocol::Swd)?;
    
    // 3. Attach to the chip using the specific `probe.attach()` method.
    let chip_name = "STM32C011F6"; // Replace with your target chip name
    println!("Attaching to chip: {}", chip_name);
    let mut session: Session = probe.attach(chip_name, Permissions::default())
        .context(format!("Failed to attach to the chip '{}'", chip_name))?;

    println!("Successfully attached to the target.");

    // 4. Access the core and interact with the target (e.g., halt it).
    let mut core = session.core(0)?;

    core.reset()?;
    println!("Resetting core.");
    sleep(time::Duration::from_millis(10));
    
    // enable writing to flash
    core.write_word_32(0x40022008, 0x45670123)?;
    core.write_word_32(0x40022008, 0xCDEF89AB)?;
    // Unlock option bytes
    core.write_word_32(0x4002200C, 0x08192A3B)?;
    core.write_word_32(0x4002200C, 0x4C5D6E7F)?;

    // Write new option byte to enable BOOT0
    core.write_word_32(0x40022020, 0xFEFFFFAA)?;
    core.write_word_32(0x40022014, 0x00020000)?;

    // Lock option bytes
    core.write_word_32(0x40022014, 0xC0000000)?;

       // Read option bytes register
    let word = core.read_word_32(0x1FFF7800)?;
    
    if word == 0xFEFFFFAA {
        println!("Option bytes successfully set to: 0x{:x}", word);        
    } else {
        anyhow::bail!("Failed to set option bytes.");
    }

    Ok(())
}