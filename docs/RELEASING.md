# Firmware releases

GitHub Actions builds tagged releases from source, records the toolchain and
memory use, generates SHA-256 checksums, creates provenance attestations and
opens a draft GitHub Release. Publishing the draft remains a deliberate human
step after reviewing the hardware-validation record.

## Prepare a release

1. Choose one semantic version for every target in the release, for example
   `4.2.0`. `SOFTWARE_VERSION`, the README and the tag must agree.
2. Create `docs/releases/vX.Y.Z-validation.md` from the previous release's
   structure. Record the boards and exact behaviour physically tested.
3. Complete the normal pull-request and Firmware CI process. Regenerate every
   affected tracked artifact from the exact proposed source tree.
4. Change `Release status: Pending` to `Release status: Ready` only after the
   validation record is accurate and any release-blocking test has passed.
5. Merge the release preparation through the protected `master` branch.

CI success is not hardware validation. A target may be marked build-only or not
tested, but that limitation must be explicit in the validation record and
release notes.

## Create the tag

Update local `master`, then create and push an annotated tag:

```sh
git switch master
git pull --ff-only origin master
git tag -a v4.2.0 -m "NZHS Annealer 4.2.0"
git push origin v4.2.0
```

The release workflow accepts tags shaped like `vX.Y.Z`, verifies that the tag
commit is on `origin/master`, confirms the firmware version and validation file,
and rebuilds all supported targets with the pinned CI toolchain.

Do not move or reuse a release tag after it has been pushed. Correct a failed or
incorrect release with a new patch version.

## Draft release contents

- Uno R3 standard HEX
- Uno R3 HEX with bootloader
- Uno R4 Minima BIN
- Uno R4 WiFi BIN
- `SHA256SUMS`
- `BUILD-INFO.txt` containing the commit, toolchain, libraries and memory use
- `HARDWARE-VALIDATION.md`, copied from the versioned validation record

The workflow builds these files afresh. It also compares the new R3 and R4 WiFi
images with the tracked artifacts before creating the draft.

Review the draft assets, release notes and validation limitations before
selecting **Publish release** on GitHub.

## Verify downloaded firmware

After downloading all release files into one directory, verify their checksums:

```sh
shasum -a 256 -c SHA256SUMS
```

With GitHub CLI installed, verify that a firmware image was built by this
repository's release workflow:

```sh
gh attestation verify NZHS_ANNEALER_128x32_OLED.ino.standard.hex \
  --repo nickb834/NZHS_ANNEALER
```

Provenance identifies the source repository, commit and GitHub Actions build. It
does not certify the electrical design, hardware tests or safe operation of the
annealer.
