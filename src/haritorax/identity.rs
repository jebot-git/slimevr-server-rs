//! SlimeTora-compatible stable identities, reused from Shora.

/// Derive the stable 6-byte MAC from a tracker name, exactly matching SlimeTora's
/// `MacAddressFromName` (the `rand-seed` npm package: FNV-1a-seeded **sfc32**).
///
/// Matching SlimeTora's derivation means a user's existing SlimeVR server config
/// already recognises these trackers — no re-acceptance in the GUI is needed.
pub fn mac_from_name(name: &str) -> [u8; 6] {
    let mut rng = Sfc32::new(name);
    let mut out = [0u8; 6];
    for byte in &mut out {
        *byte = (rng.next_u32() >> 24) as u8;
    }
    out
}

/// FNV-1a 32-bit hash with JS `Math.imul` (signed 32-bit multiply) semantics.
fn fnv1a_32(s: &str) -> i32 {
    let mut t: i32 = 0x811c_9dc5u32 as i32; // 2166136261
    for cu in s.encode_utf16() {
        t = (t ^ (cu as i32)).wrapping_mul(16777619);
    }
    t
}

/// One step of the `rand-seed` hash mixer. Returns `(next_running_state, u32)`.
fn hash_mix(t: i64) -> (i64, u32) {
    let mut t = t;
    t += ((t as i32) << 13) as i64;
    t = ((t as i32) ^ (((t as u32) >> 7) as i32)) as i64;
    t += ((t as i32) << 3) as i64;
    t = ((t as i32) ^ (((t as u32) >> 17) as i32)) as i64;
    t += ((t as i32) << 5) as i64;
    (t, t as u32)
}

/// The `sfc32` PRNG used by `rand-seed` (default generator).
struct Sfc32 {
    a: u32,
    b: u32,
    c: u32,
    d: u32,
}

impl Sfc32 {
    fn new(seed: &str) -> Self {
        let mut h = fnv1a_32(seed) as i64;
        let mut state = [0u32; 4];
        for slot in &mut state {
            let (next, v) = hash_mix(h);
            h = next;
            *slot = v;
        }
        Self {
            a: state[0],
            b: state[1],
            c: state[2],
            d: state[3],
        }
    }

    fn next_u32(&mut self) -> u32 {
        let t = self.a.wrapping_add(self.b).wrapping_add(self.d);
        self.a = self.b ^ (self.b >> 9);
        self.b = self.c.wrapping_add(self.c << 3);
        self.c = (self.c << 21 | self.c >> 11).wrapping_add(t);
        self.d = self.d.wrapping_add(1);
        t
    }
}

#[cfg(test)]
mod tests {
    use super::mac_from_name;

    fn hex(mac: [u8; 6]) -> String {
        mac.iter()
            .map(|b| format!("{b:02X}"))
            .collect::<Vec<_>>()
            .join(":")
    }

    #[test]
    fn mac_matches_slimetora() {
        // Known values from a real SlimeTora-generated SlimeVR config.
        assert_eq!(hex(mac_from_name("hip")), "91:E8:47:F2:CB:A6");
        assert_eq!(hex(mac_from_name("rightKnee")), "57:F3:1B:A0:D4:BC");
        assert_eq!(hex(mac_from_name("leftAnkle")), "7E:E2:E4:79:6A:A8");
    }
}
