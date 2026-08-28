//! Save a file from the UI. Native pops a "Save As" dialog; wasm triggers a
//! browser download via a Blob + anchor click.

/// The MIME type a browser download is labelled with. Native ignores it — the
/// name and the bytes are all a filesystem needs — but a browser hands the file
/// to whatever the type says, so a picture saved as `text/plain` opens in a text
/// editor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mime {
    Text,
    Png,
}

impl Mime {
    #[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))] // only the browser labels a download
    fn as_str(self) -> &'static str {
        match self {
            Mime::Text => "text/plain",
            Mime::Png => "image/png",
        }
    }
}

/// Save `data` under a suggested `name`, as a text file.
pub fn save(name: &str, data: &[u8]) {
    save_as(name, data, Mime::Text);
}

/// Save `data` under a suggested `name`, labelled as `mime`.
#[cfg(not(target_arch = "wasm32"))]
pub fn save_as(name: &str, data: &[u8], _mime: Mime) {
    let data = data.to_vec();
    let name = name.to_string();
    // rfd's dialog is blocking; run it off the UI thread.
    std::thread::Builder::new()
        .name("sdroxide-save".into())
        .spawn(move || {
            if let Some(path) = rfd::FileDialog::new().set_file_name(&name).save_file() {
                if let Err(e) = std::fs::write(&path, &data) {
                    eprintln!("sdroxide: saving {}: {e}", path.display());
                }
            }
        })
        .ok();
}

/// A text file as it was read.
pub struct Loaded {
    pub text: String,
    /// The code page the bytes had to be *guessed* to be in, when the file did
    /// not say and was not UTF-8. `None` when nothing was guessed. Worth showing
    /// the operator: it is the one part of the read that could be wrong.
    pub assumed: Option<&'static str>,
}

/// Where a [`load_text`] pick is delivered: the file it read, or why it could
/// not be read. Stays `None` when the operator cancels the dialog.
pub type LoadInbox = std::sync::Arc<std::sync::Mutex<Option<Result<Loaded, String>>>>;

/// Open a text file via a native "Open" dialog (off the UI thread) and store
/// its contents into `inbox` for the UI to pick up next frame. Native only —
/// the browser client has no filesystem picker here.
#[cfg(not(target_arch = "wasm32"))]
pub fn load_text(filter_name: &str, ext: &str, inbox: LoadInbox) {
    let filter_name = filter_name.to_string();
    let ext = ext.to_string();
    std::thread::Builder::new()
        .name("sdroxide-open".into())
        .spawn(move || {
            let Some(path) =
                rfd::FileDialog::new().add_filter(&filter_name, &[ext.as_str()]).pick_file()
            else {
                return;
            };
            // Read bytes, not a `String`: `read_to_string` refuses a file that
            // is not valid UTF-8 outright, and an operator's log exported by a
            // Windows logger very often is not. Refusing it lost the whole
            // import — every callsign, date and band in it, all of them plain
            // ASCII — over a code page in one name field.
            let outcome = match std::fs::read(&path) {
                Ok(bytes) => Ok(decode_text(&bytes)),
                Err(e) => Err(format!("reading {}: {e}", path.display())),
            };
            if let Ok(mut g) = inbox.lock() {
                *g = Some(outcome);
            }
        })
        .ok();
}

// Native only, with `load_text`: the browser hands its own decoded string to
// the wasm client, so none of this is reachable there.
#[cfg(not(target_arch = "wasm32"))]
/// Decode the bytes of a text file, saying what — if anything — had to be
/// assumed to do it.
///
/// A byte-order mark settles the question outright, and so does the file simply
/// being valid UTF-8: nothing else looks like UTF-8 by accident for more than a
/// few bytes. What is left is a legacy single-byte code page, which no file
/// declares and no reader can be certain of, so one is picked by
/// [`looks_cyrillic`] and named in the return so the operator can see the guess.
///
/// Anything is better than nothing here. The fields that matter most in an ADIF
/// log — callsign, date, band, mode, frequency — are ASCII under every one of
/// these encodings, so even a wrongly guessed code page costs at worst the
/// spelling of a name, where refusing the file costs the whole log.
pub fn decode_text(bytes: &[u8]) -> Loaded {
    let unicode = |text: String| Loaded { text, assumed: None };
    if let Some(rest) = bytes.strip_prefix(&[0xef, 0xbb, 0xbf]) {
        return unicode(String::from_utf8_lossy(rest).into_owned());
    }
    if let Some(rest) = bytes.strip_prefix(&[0xff, 0xfe]) {
        return unicode(from_utf16(rest, u16::from_le_bytes));
    }
    if let Some(rest) = bytes.strip_prefix(&[0xfe, 0xff]) {
        return unicode(from_utf16(rest, u16::from_be_bytes));
    }
    if let Ok(text) = std::str::from_utf8(bytes) {
        return unicode(text.to_string());
    }
    let (name, table): (_, fn(u8) -> char) = if looks_cyrillic(bytes) {
        ("Windows-1251", sdroxide_types::text::cp1251_char)
    } else {
        ("Windows-1252", sdroxide_types::text::cp1252_char)
    };
    Loaded { text: bytes.iter().map(|&b| table(b)).collect(), assumed: Some(name) }
}

#[cfg(not(target_arch = "wasm32"))]
/// UTF-16 code units in the order `pair` reads them, surrogates paired up. A
/// lone surrogate or a trailing odd byte is a truncated file, not a reason to
/// throw the rest away.
fn from_utf16(bytes: &[u8], pair: fn([u8; 2]) -> u16) -> String {
    let units = bytes.chunks_exact(2).map(|c| pair([c[0], c[1]]));
    char::decode_utf16(units).map(|c| c.unwrap_or(char::REPLACEMENT_CHARACTER)).collect()
}

#[cfg(not(target_arch = "wasm32"))]
/// Whether a file that is not Unicode reads better as Cyrillic than as Western
/// European.
///
/// The two code pages overlap byte for byte — 0xE0 is `à` in one and `а` in the
/// other — so no single value can tell them apart. The shape of the text can:
/// Cyrillic spells a whole word out of high bytes, so its runs are as long as
/// its words, while Western European text is ASCII with an accent dropped into
/// it here and there. Three high bytes in a row is a word in one and a spelling
/// nobody uses in the other, so that is where the line is drawn. Two is not
/// enough — Finnish and Estonian really do write `ää`.
fn looks_cyrillic(bytes: &[u8]) -> bool {
    let (mut run, mut longest) = (0usize, 0usize);
    for &b in bytes {
        run = if b >= 0x80 { run + 1 } else { 0 };
        longest = longest.max(run);
    }
    longest >= 3
}

#[cfg(target_arch = "wasm32")]
pub fn save_as(name: &str, data: &[u8], mime: Mime) {
    use wasm_bindgen::JsCast;

    let array = js_sys::Uint8Array::from(data);
    let parts = js_sys::Array::new();
    parts.push(&array.buffer());
    let opts = web_sys::BlobPropertyBag::new();
    opts.set_type(mime.as_str());
    let blob = match web_sys::Blob::new_with_u8_array_sequence_and_options(&parts, &opts) {
        Ok(b) => b,
        Err(_) => return,
    };
    let Ok(url) = web_sys::Url::create_object_url_with_blob(&blob) else { return };

    let Some(doc) = web_sys::window().and_then(|w| w.document()) else { return };
    if let Ok(a) = doc.create_element("a") {
        let a: web_sys::HtmlAnchorElement = a.unchecked_into();
        a.set_href(&url);
        a.set_download(name);
        a.click();
    }
    let _ = web_sys::Url::revoke_object_url(&url);
}

#[cfg(all(test, not(target_arch = "wasm32")))]
mod tests {
    use super::*;

    /// Plain ASCII, and UTF-8 that says so, are read as themselves with nothing
    /// assumed — the case that must not regress into a guess.
    #[test]
    fn unicode_is_read_as_itself() {
        let plain = decode_text(b"<CALL:4>W1AW<EOR>");
        assert_eq!(plain.text, "<CALL:4>W1AW<EOR>");
        assert_eq!(plain.assumed, None);

        let utf8 = decode_text("<NAME:9>Владимир".as_bytes());
        assert_eq!(utf8.text, "<NAME:9>Владимир");
        assert_eq!(utf8.assumed, None);

        // A BOM belongs to the encoding, not to the text: left in, it would be
        // the first character of the first tag.
        let mut bom = vec![0xef, 0xbb, 0xbf];
        bom.extend_from_slice("<CALL:4>W1AW".as_bytes());
        assert_eq!(decode_text(&bom).text, "<CALL:4>W1AW");

        let utf16: Vec<u8> = [0xff, 0xfe]
            .into_iter()
            .chain("<CALL:4>W1AW".encode_utf16().flat_map(|u| u.to_le_bytes()))
            .collect();
        let read = decode_text(&utf16);
        assert_eq!(read.text, "<CALL:4>W1AW");
        assert_eq!(read.assumed, None);
    }

    /// The issue this was written for: a log exported in a national code page
    /// used to import as nothing at all.
    #[test]
    fn a_cyrillic_code_page_still_imports() {
        // "<NAME:8>Владимир <QTH:6>Москва <EOR>" in Windows-1251.
        let mut raw = b"<NAME:8>".to_vec();
        raw.extend_from_slice(&[0xc2, 0xeb, 0xe0, 0xe4, 0xe8, 0xec, 0xe8, 0xf0]);
        raw.extend_from_slice(b" <QTH:6>");
        raw.extend_from_slice(&[0xcc, 0xee, 0xf1, 0xea, 0xe2, 0xe0]);
        raw.extend_from_slice(b" <EOR>");
        let read = decode_text(&raw);
        assert_eq!(read.assumed, Some("Windows-1251"));
        assert_eq!(read.text, "<NAME:8>Владимир <QTH:6>Москва <EOR>");
        // And the declared lengths, counted in the code page's own bytes, still
        // land: the parser reads them as characters once they no longer fit as
        // UTF-8 bytes.
        let recs = sdroxide_types::adif_to_qso_log(&format!("<CALL:5>UA1AB {}", read.text));
        assert_eq!(recs.len(), 1);
        assert_eq!(recs[0].name, "Владимир");
        assert_eq!(recs[0].qth, "Москва");
    }

    /// The other direction: an accent dropped into ASCII is Western European,
    /// and reading it as Cyrillic would turn every one of them into a letter
    /// from the wrong alphabet.
    #[test]
    fn a_western_code_page_is_not_mistaken_for_cyrillic() {
        // "<NAME:5>Jörg <QTH:9>Jyväskylä <EOR>" in Windows-1252.
        let mut raw = b"<NAME:5>J".to_vec();
        raw.push(0xf6);
        raw.extend_from_slice(b"rg <QTH:9>Jyv");
        raw.push(0xe4);
        raw.extend_from_slice(b"skyl");
        raw.push(0xe4);
        raw.extend_from_slice(b" <EOR>");
        let read = decode_text(&raw);
        assert_eq!(read.assumed, Some("Windows-1252"));
        assert_eq!(read.text, "<NAME:5>Jörg <QTH:9>Jyväskylä <EOR>");
    }
}
