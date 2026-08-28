//! Legacy single-byte text, decoded rather than dropped.
//!
//! Amateur radio is full of text that predates Unicode and says so nowhere: an
//! ADIF log written by a Windows program, a packet BBS whose software is older
//! than UTF-8, an RDS broadcast with its own character table. The house rule
//! for all of them is the same — anything is better than nothing. A wrongly
//! guessed code page costs the spelling of a name; refusing the bytes costs the
//! whole log, or the whole session.
//!
//! Lives here rather than beside its first caller because it now has two on
//! opposite sides of the tree: the ADIF importer in the UI crate, and the
//! packet terminal in `sdroxide-digi`.

/// Windows-1252 (the Western European code page, and what "ANSI" means on a
/// Western Windows). Its 0xA0-0xFF half is Latin-1 outright; only the 0x80-0x9F
/// block is its own, and that block is typography rather than letters. The five
/// positions Microsoft never assigned come back as the replacement character.
#[rustfmt::skip]
#[must_use]
pub fn cp1252_char(b: u8) -> char {
    const HIGH: [char; 32] = [
        '€', '\u{fffd}', '‚', 'ƒ', '„', '…', '†', '‡', 'ˆ', '‰', 'Š', '‹', 'Œ', '\u{fffd}', 'Ž', '\u{fffd}',
        '\u{fffd}', '‘', '’', '“', '”', '•', '–', '—', '˜', '™', 'š', '›', 'œ', '\u{fffd}', 'ž', 'Ÿ',
    ];
    match b {
        0x80..=0x9f => HIGH[(b - 0x80) as usize],
        _ => b as char,
    }
}

/// Windows-1251 (the Cyrillic code page). Its top four rows are the Russian
/// alphabet laid out in Unicode's own order, so those are arithmetic; the
/// 0x80-0xBF half mixes the same typography as 1252 with the letters the other
/// Cyrillic languages add, and that part is a table.
#[rustfmt::skip]
#[must_use]
pub fn cp1251_char(b: u8) -> char {
    const HIGH: [char; 64] = [
        'Ђ', 'Ѓ', '‚', 'ѓ', '„', '…', '†', '‡', '€', '‰', 'Љ', '‹', 'Њ', 'Ќ', 'Ћ', 'Џ',
        'ђ', '‘', '’', '“', '”', '•', '–', '—', '\u{fffd}', '™', 'љ', '›', 'њ', 'ќ', 'ћ', 'џ',
        '\u{a0}', 'Ў', 'ў', 'Ј', '¤', 'Ґ', '¦', '§', 'Ё', '©', 'Є', '«', '¬', '\u{ad}', '®', 'Ї',
        '°', '±', 'І', 'і', 'ґ', 'µ', '¶', '·', 'ё', '№', 'є', '»', 'ј', 'Ѕ', 'ѕ', 'ї',
    ];
    match b {
        0x80..=0xbf => HIGH[(b - 0x80) as usize],
        // 0xC0 is А and the alphabet runs unbroken from there, upper case then
        // lower, exactly as U+0410 onwards does.
        0xc0..=0xff => char::from_u32(0x410 + (b - 0xc0) as u32).unwrap_or(char::REPLACEMENT_CHARACTER),
        _ => b as char,
    }
}

/// Read bytes as text, preferring Unicode and falling back to Windows-1252.
///
/// For a stream that carries no declaration and no byte-order mark, which is
/// every AX.25 information field: valid UTF-8 is taken at face value — nothing
/// else looks like UTF-8 by accident for more than a few bytes — and anything
/// else is read as the code page most of this traffic was typed in.
#[must_use]
pub fn decode_cp1252(bytes: &[u8]) -> String {
    match std::str::from_utf8(bytes) {
        Ok(text) => text.to_string(),
        Err(_) => bytes.iter().map(|&b| cp1252_char(b)).collect(),
    }
}

/// Write text as Windows-1252, replacing what the code page cannot say.
///
/// The other direction, for a link whose far end predates Unicode: a BBS handed
/// a two-byte UTF-8 sequence prints two characters of nonsense where one
/// Windows-1252 byte would have been the right letter. Anything outside the
/// code page becomes `?`, which is what every legacy gateway does with it and
/// what an operator can see and retype.
#[must_use]
pub fn encode_cp1252(text: &str) -> Vec<u8> {
    text.chars()
        .map(|c| {
            if (c as u32) < 0x80 {
                return c as u8;
            }
            // The high half is small enough to search: 128 positions, once per
            // non-ASCII character typed by a human.
            (0x80u8..=0xff).find(|&b| cp1252_char(b) == c).unwrap_or(b'?')
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ascii_is_itself_in_both_directions() {
        let s = "de OE3JJS-10 > help";
        assert_eq!(decode_cp1252(s.as_bytes()), s);
        assert_eq!(encode_cp1252(s), s.as_bytes());
    }

    /// The case the whole module exists for: a byte a BBS really sends, which
    /// `from_utf8_lossy` would turn into a replacement character.
    #[test]
    fn a_latin_1_byte_is_a_letter_not_a_question_mark() {
        assert_eq!(decode_cp1252(&[b'G', b'r', 0xfc, 0xdf, b'e']), "Grüße");
    }

    #[test]
    fn utf8_wins_where_the_bytes_are_valid_utf8() {
        assert_eq!(decode_cp1252("Grüße".as_bytes()), "Grüße");
    }

    #[test]
    fn the_round_trip_holds_for_what_the_code_page_can_say() {
        let s = "Grüße aus Österreich — 73";
        assert_eq!(decode_cp1252(&encode_cp1252(s)), s);
    }

    /// What it cannot say becomes something an operator can see and retype,
    /// rather than a byte the far end reads as a control character.
    #[test]
    fn what_the_code_page_cannot_say_becomes_a_question_mark() {
        assert_eq!(encode_cp1252("ok — до свидания"), b"ok \x97 ?? ????????".to_vec());
    }
}
