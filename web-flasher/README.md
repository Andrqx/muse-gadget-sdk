# Web flasher

A page on GitHub Pages that flashes Muse Gadget firmware from Chrome or Edge
over Web Serial, using [ESP Web Tools](https://esphome.github.io/esp-web-tools/).
The ESP32 CI (`.github/workflows/esp32.yml`) packages the boards listed in
`boards.json` from its own builds, assembles the site on every run, and deploys
it on pushes to `main`.

## Adding a board

Add an entry to `boards.json`. `board` must match a `board` in the build
matrix in `esp32.yml`; the site job fails if it doesn't.

| Field    | Required | Meaning |
|----------|----------|---------|
| `board`  | yes      | Board id, as in the build matrix. |
| `name`   | yes      | Name shown in the board picker. |
| `pacing` | no       | Paced serial writes for USB bridges that drop bytes, as `{"chunk": 32, "factor": 2}`: bytes per write, and how many times as long as a chunk takes on the wire to wait after it. Leave it out for boards that flash fine without. |
| `note`   | no       | HTML shown under the picker when the board is selected: which port to pick, backups to take first, and so on. |

## Testing locally

Download the `github-pages` artifact from an ESP32 CI run, unpack
`artifact.tar` and serve it on localhost; Chrome allows Web Serial there
without https:

```sh
gh run download RUN_ID -n github-pages -D /tmp/flasher
mkdir /tmp/flasher/site && tar -xf /tmp/flasher/artifact.tar -C /tmp/flasher/site
python3 -m http.server 8000 --bind 127.0.0.1 --directory /tmp/flasher/site
```

To try changes to the page itself, copy `index.html`, `paced-serial.js` and
`boards.json` over the unpacked site and reload.
