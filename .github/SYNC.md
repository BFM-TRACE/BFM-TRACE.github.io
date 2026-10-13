# Publish the TRACE homepage

This repository mirrors the tracked files in the original project's `homepage/`
directory. GitHub Actions validates the source, builds `dist/`, and publishes only
that static bundle at <https://bfm-trace.github.io/>.

The homepage, videos, paper, browser simulation, models, and licenses are copied
without changing their contents. The `.github/` directory contains the additional
publishing and synchronization configuration. `dist/` and new QA reports remain
ignored by Git.

## Synchronize a source update

Use the original homepage as the source of truth. Add new source files to its Git
index first so they are included by the sync. Existing tracked files can be
synchronized before or after committing their edits in the source repository.

From this publishing repository:

```sh
python3 .github/scripts/sync-homepage.py ../BFM-Trace/homepage
npm run check:release
git add -A
git commit -m "Sync TRACE homepage"
git push origin main
```

For another checkout location, pass its homepage directory explicitly. The sync
removes only files listed by an earlier sync that no longer exist in the source;
it preserves `.github/` and unrelated destination files.

The source file hashes and source commit are recorded in
`.github/homepage-source.json`. The build produces `dist/build-manifest.json`,
which can be compared with the local site's manifest to check publication parity.

## GitHub Pages setting

In repository Settings > Pages, choose **GitHub Actions** as the publishing source.
Leave the custom domain unset to keep `bfm-trace.github.io` in the address bar.
Every push to `main` triggers validation and deployment. A manual run is also
available from Actions > Publish TRACE homepage > Run workflow.
