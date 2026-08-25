#!/usr/bin/env bash
# Build a real, double-clickable sdroxide.app for local use on macOS.
#
# Extracted from the "cat" (non-SoapySDR) release variant's own bundling
# steps in .github/workflows/release.yml -- until now the *only* place this
# logic existed, which is why every local build handed out during
# development has been the bare `target/release/sdroxide` executable
# rather than a real bundle. Everything downstream of the release binary
# here -- Info.plist, the icon, the license notices, the ad-hoc codesign --
# matches that CI job exactly; the .dmg wrapping step is left out on
# purpose, since a local build has nowhere to distribute a disk image to.
# Drag the .app to /Applications yourself, or just double-click it in
# place.
#
#   packaging/macos/build-app.sh                # build + bundle
#   packaging/macos/build-app.sh --no-build      # bundle only, reuse
#                                                 # whatever is already in
#                                                 # target/release
#   packaging/macos/build-app.sh --with-web      # also embed the web
#                                                 # client, like the real
#                                                 # release does (needs
#                                                 # `trunk`; see below)
#
# Matches the bundled release variant's own feature set: no SoapySDR
# (--no-default-features --features rtl433), so the app needs no
# libSoapySDR installed and nothing to dlopen at first launch -- exactly
# the "cat" variant's own reason for existing (see the workflow's comment
# by matrix.variant). SoapySDR users who want that support back should run
# `cargo build --release` (the plain default-features build, includes
# soapy) and use the resulting target/release/sdroxide directly instead of
# this script -- the real release does the same, shipping SoapySDR users a
# portable tarball rather than an .app.
#
# The web client is left out unless --with-web is passed: the real release
# embeds it via `trunk build --release` in crates/sdroxide-web (a Rust/WASM
# project, not npm), run once by CI and shared across every native build --
# a plain checkout has neither `trunk` nor the wasm32 target installed, and
# the native GUI this script exists to package doesn't need it. Pass
# --with-web to build it here too; the script tells you what to install if
# `trunk` is missing rather than failing unhelpfully partway through.
set -euo pipefail

repo=$(cd "$(dirname "$0")/../.." && pwd)
cd "$repo"

# A plain rustup install adds this to the shell profile, but a script run
# from a non-interactive/non-login shell (an IDE task, a cron job, some
# sandboxed tool) can start without ever sourcing it -- checked for
# directly rather than assumed, so this script fails with a real build
# rather than a confusing "cargo: command not found" on exactly the
# machines it's most likely to matter on.
if ! command -v cargo >/dev/null && [[ -x "$HOME/.cargo/bin/cargo" ]]; then
  export PATH="$HOME/.cargo/bin:$PATH"
fi

do_build=1
do_web=0
for arg in "$@"; do
  case "$arg" in
    --no-build) do_build=0 ;;
    --with-web) do_web=1 ;;
    -h | --help)
      sed -n '2,40p' "$0" | sed 's/^# \{0,1\}//'
      exit 0
      ;;
    *)
      echo "usage: $0 [--no-build] [--with-web]" >&2
      exit 1
      ;;
  esac
done

if [[ $do_web -eq 1 && ! -s crates/sdroxide-web/dist/index.html ]]; then
  if ! command -v trunk >/dev/null; then
    echo "--with-web needs trunk (cargo install trunk) and the wasm32-unknown-unknown target" >&2
    echo "  (rustup target add wasm32-unknown-unknown), neither of which is installed." >&2
    exit 1
  fi
  echo "Building the web client (trunk build --release)..."
  (cd crates/sdroxide-web && trunk build --release)
fi

if [[ $do_build -eq 1 ]]; then
  echo "Building the release binary (no SoapySDR, matching the bundled release variant)..."
  if [[ $do_web -eq 1 ]]; then
    cargo build --release --locked -p sdroxide --no-default-features --features embed-web,rtl433
  else
    cargo build --release --locked -p sdroxide --no-default-features --features rtl433
  fi
fi

[[ -x target/release/sdroxide ]] || {
  echo "target/release/sdroxide not found -- build it first, or drop --no-build" >&2
  exit 1
}

version=$(sed -n 's/^version *= *"\([^"]*\)".*/\1/p' Cargo.toml | head -n1)

out=build/macos-app
app="$out/sdroxide.app"
rm -rf "$app"
mkdir -p "$app/Contents/MacOS" "$app/Contents/Resources"

sed "s/@VERSION@/$version/g" packaging/macos/Info.plist >"$app/Contents/Info.plist"
cp target/release/sdroxide "$app/Contents/MacOS/sdroxide"
# CFBundleIconFile=sdroxide resolves to this; without it Launchpad and the
# Dock fall back to the blank generic-application icon.
cp packaging/macos/sdroxide.icns "$app/Contents/Resources/sdroxide.icns"

# License notices for the embedded firmware/model/voice this binary
# carries -- copied into the bundle (so they survive dragging the .app out
# of this directory) before codesign, which seals the contents of
# Resources into the signature.
cp crates/sdroxide-rx888/firmware/LICENSE.txt "$app/Contents/Resources/LICENSE-rx888-firmware.txt"
cp crates/sdroxide-speech/assets/LICENSE-cmudict.txt "$app/Contents/Resources/"
cp crates/sdroxide-deepcw/LICENSE "$app/Contents/Resources/LICENSE-deepcw.txt"
mkdir -p "$app/Contents/Resources/voices"
cp assets/voices/en_US-hfc_female-medium.onnx \
  assets/voices/en_US-hfc_female-medium.onnx.json \
  assets/voices/en_US-hfc_female-medium.MODEL_CARD \
  "$app/Contents/Resources/voices/"
cp assets/voices/LICENSE-voice.txt "$app/Contents/Resources/"

# Ad-hoc signature (no Apple Developer ID here) -- without this, Gatekeeper
# refuses to launch the bundle outright rather than just warning. Binary
# first, then the bundle as a whole, matching the order that actually
# produces a signature `codesign --verify --strict` accepts. A build
# straight out of this script carries no quarantine flag (that only gets
# set on something downloaded through a browser or AirDrop), so ad-hoc
# signing alone is enough for a local double-click to work.
codesign --force -s - "$app/Contents/MacOS/sdroxide"
codesign --force -s - "$app"

echo
echo "Built $app ($version)."
echo "Drag it to /Applications, or double-click it in place."
