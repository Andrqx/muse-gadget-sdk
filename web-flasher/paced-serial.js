// Copyright (c) Meta Platforms, Inc. and affiliates.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Paced Web Serial writes, the browser's version of esp32/tools/muse/paced_esptool.py.
//
// Some USB-UART bridges (the SenseCAP Watcher's CH342) have no flow control
// and drop bytes when a whole packet arrives faster than the UART drains it,
// so flashing fails while uploading the stub or on the first flash block.
// While pacing is on, every SerialPort's writable stream sends 64 bytes at a
// time at the line rate. esptool-js, inside ESP Web Tools, takes a writer from
// port.writable for each write, so it picks this up without changes.

const CHUNK = 64;
const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

export function installPacedSerial(SerialPortClass) {
  const proto = SerialPortClass.prototype;
  const writable = Object.getOwnPropertyDescriptor(proto, "writable");
  const open = proto.open;
  const baudRates = new WeakMap(); // port -> baud rate it was opened at
  const paced = new WeakMap(); // the port's own writable -> its paced wrapper
  let enabled = false;

  proto.open = function (options) {
    baudRates.set(this, options.baudRate);
    return open.call(this, options);
  };

  Object.defineProperty(proto, "writable", {
    configurable: true,
    get() {
      const real = writable.get.call(this);
      if (!enabled || !real) return real;
      let stream = paced.get(real);
      if (!stream) {
        const port = this;
        // Lock the port's stream only while writing, so port.close() can run
        // between writes.
        const withWriter = async (fn) => {
          const writer = real.getWriter();
          try {
            await fn(writer);
          } finally {
            writer.releaseLock();
          }
        };
        stream = new WritableStream({
          write: (data) =>
            withWriter(async (writer) => {
              const bytes = new Uint8Array(data);
              for (let i = 0; i < bytes.length; i += CHUNK) {
                const chunk = bytes.subarray(i, i + CHUNK);
                await writer.write(chunk);
                // 10 bits a byte on the wire, rounded up: Web Serial can't wait
                // for the UART to drain the way pyserial's flush() does.
                await sleep(Math.ceil((chunk.length * 10 * 1000) / (baudRates.get(port) || 115200)));
              }
            }),
          close: () => withWriter((writer) => writer.close()),
          abort: (reason) => real.abort(reason),
        });
        paced.set(real, stream);
      }
      return stream;
    },
  });

  return {
    setEnabled(on) {
      enabled = on;
    },
  };
}
