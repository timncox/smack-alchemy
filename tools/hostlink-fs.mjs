#!/usr/bin/env node
/**
 * hostlink-fs — the SD card over HostLink, from a terminal.
 *
 * The SDK's hostlink-cli covers presets and reboot; this covers the
 * filesystem block (protocol §8, commands 0x50–0x5B) so firmware images can
 * reach the card's /alchemy folder without pulling the card. Same wire
 * codecs as lib/alchemy-sdk/tools/hostlink-cli/hostlink.mjs (copied, since
 * that file is a script, not a module). Dependency-free; macOS / Linux.
 *
 *   node tools/hostlink-fs.mjs [-p /dev/cu.usbmodemXXXX] info
 *   node tools/hostlink-fs.mjs ls /
 *   node tools/hostlink-fs.mjs stat /alchemy/smack_alchemy.bin
 *   node tools/hostlink-fs.mjs mkdir /alchemy
 *   node tools/hostlink-fs.mjs put build/smack_alchemy.bin /alchemy/smack_alchemy.bin [--overwrite]
 *   node tools/hostlink-fs.mjs get /alchemy/x.bin x.bin
 *   node tools/hostlink-fs.mjs rm /alchemy/x.bin
 *
 * Uploads are staged by the firmware into <path>.part and renamed onto the
 * final name only after a CRC-verified commit, so an interrupted put never
 * leaves a torn file under a real name.
 */
import { execFileSync } from "node:child_process";
import { readdirSync, readFileSync, writeFileSync, openSync, readSync, writeSync, closeSync } from "node:fs";

const PROTO = 1, RESP_FLAG = 0x80, ERR_TYPE = 0xff;
const CMD = { hello: 0x01, info: 0x50, list: 0x51, stat: 0x52, openRead: 0x53, read: 0x54, close: 0x55,
              openWrite: 0x56, write: 0x57, commit: 0x58, del: 0x59, mkdir: 0x5a, rename: 0x5b };
const STATUS = ["ok", "unsupported", "bad-args", "bad-state", "bad-crc", "bad-slot", "too-large",
                "schema-mismatch", "flash-fail", "busy", "frame-error",
                "fs-no-card", "fs-no-file", "fs-exists", "fs-locked", "fs-full", "fs-io"];
const FS_NO_CARD = 11;

const CRC_TABLE = (() => {
  const t = new Uint32Array(256);
  for (let i = 0; i < 256; i++) { let c = i; for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1; t[i] = c >>> 0; }
  return t;
})();
function crc32(buf, seed = 0) {
  let crc = ~seed >>> 0;
  for (let i = 0; i < buf.length; i++) crc = (CRC_TABLE[(crc ^ buf[i]) & 0xff] ^ (crc >>> 8)) >>> 0;
  return ~crc >>> 0;
}
function cobsEncode(input) {
  const out = Buffer.alloc(input.length + Math.ceil(input.length / 254) + 1);
  let o = 0, codeAt = o++, code = 1;
  for (let i = 0; i < input.length; i++) {
    if (input[i] === 0) { out[codeAt] = code; codeAt = o++; code = 1; }
    else { out[o++] = input[i]; if (++code === 0xff) { out[codeAt] = code; codeAt = o++; code = 1; } }
  }
  out[codeAt] = code;
  return out.subarray(0, o);
}
function cobsDecode(input) {
  if (input.length === 0) return null;
  const out = Buffer.alloc(input.length);
  let o = 0, i = 0;
  while (i < input.length) {
    const code = input[i++];
    if (code === 0) return null;
    const run = code - 1;
    if (i + run > input.length) return null;
    for (let k = 0; k < run; k++) { const b = input[i++]; if (b === 0) return null; out[o++] = b; }
    if (code !== 0xff && i < input.length) out[o++] = 0;
  }
  return out.subarray(0, o);
}
function buildFrame(type, seq, body) {
  const dec = Buffer.alloc(6 + body.length + 4);
  dec[0] = PROTO; dec[1] = type;
  dec.writeUInt16LE(seq, 2); dec.writeUInt16LE(body.length, 4);
  body.copy(dec, 6);
  dec.writeUInt32LE(crc32(dec.subarray(0, 6 + body.length)), 6 + body.length);
  return Buffer.concat([cobsEncode(dec), Buffer.from([0])]);
}
class FrameParser {
  constructor() { this.acc = []; }
  *push(byte) {
    if (byte !== 0) { this.acc.push(byte); return; }
    const chunk = Buffer.from(this.acc); this.acc = [];
    if (chunk.length === 0) return;
    const dec = cobsDecode(chunk);
    if (!dec || dec.length < 10) return;
    const bodyLen = dec.length - 10;
    const ok = dec[0] === PROTO && dec.readUInt16LE(4) === bodyLen &&
               crc32(dec.subarray(0, dec.length - 4)) === dec.readUInt32LE(dec.length - 4);
    yield { type: dec[1], seq: dec.readUInt16LE(2), body: dec.subarray(6, 6 + bodyLen), ok };
  }
}

function defaultPort() {
  const names = readdirSync("/dev/").filter(n => n.startsWith("cu.usbmodem") || n.startsWith("tty.usbmodem") || n.startsWith("ttyACM"));
  if (names.length === 0) throw new Error("no usbmodem/ttyACM device found; pass --port");
  return "/dev/" + names.sort()[0];
}
class SerialLink {
  constructor(port) {
    this.port = port;
    try { execFileSync("stty", ["-f", port, "raw", "-echo", "115200"], { stdio: "ignore" }); }
    catch { execFileSync("stty", ["-F", port, "raw", "-echo", "115200"], { stdio: "ignore" }); }
    this.fd = openSync(port, "r+");
    this.parser = new FrameParser(); this.seq = 1; this.buf = Buffer.alloc(8192); this.maxBody = 512;
  }
  close() { try { closeSync(this.fd); } catch { /* ignore */ } }
  /** Send a command; return the response body (status at [0]). Throws on
   *  non-OK unless `raw`, so callers can read diagnostic trailers. */
  request(type, body = Buffer.alloc(0), timeoutMs = 3000, raw = false) {
    const seq = this.seq; this.seq = (this.seq + 1) & 0xffff || 1;
    writeSync(this.fd, buildFrame(type, seq, body));
    const deadline = Date.now() + timeoutMs;
    while (Date.now() < deadline) {
      let n = 0;
      try { n = readSync(this.fd, this.buf, 0, this.buf.length, null); } catch { n = 0; }
      for (let i = 0; i < n; i++) for (const f of this.parser.push(this.buf[i])) {
        if (f.seq !== seq || !f.ok) continue;
        if (f.type === ERR_TYPE) throw new Error(`frame error: ${STATUS[f.body[0]] ?? f.body[0]}`);
        if (f.type !== (type | RESP_FLAG)) throw new Error(`unexpected type 0x${f.type.toString(16)}`);
        if (f.body[0] !== 0 && !raw) throw new Error(`device status: ${STATUS[f.body[0]] ?? f.body[0]}`);
        return f.body;
      }
    }
    throw new Error(`timeout waiting for response to 0x${type.toString(16)}`);
  }
}

const str = s => { const b = Buffer.from(s, "ascii"); if (b.length > 255) throw new Error("string too long"); return Buffer.concat([Buffer.from([b.length]), b]); };
function readStr(buf, at) { const n = buf[at]; return [buf.subarray(at + 1, at + 1 + n).toString("ascii"), at + 1 + n]; }
function u32(v) { const b = Buffer.alloc(4); b.writeUInt32LE(v >>> 0); return b; }

function hello(link) {
  const b = link.request(CMD.hello);
  link.maxBody = b.readUInt16LE(5 + 12 + 16);
  let at = 5 + 12 + 16 + 4, id, name, fw;
  [id, at] = readStr(b, at); [name, at] = readStr(b, at); [fw, at] = readStr(b, at);
  return { id, name, fw, maxBody: link.maxBody };
}

function probe(b, at) {
  return { read_result: b[at], sig: b.subarray(at + 1, at + 3).toString("hex"), first: b[at + 3],
           oem: b.subarray(at + 4, at + 12).toString("ascii"), fstype: b.subarray(at + 12, at + 20).toString("ascii") };
}
function info(link) {
  const b = link.request(CMD.info, Buffer.alloc(0), 10000, true);
  const status = b[0];
  if (status === FS_NO_CARD) {
    const out = { status: "fs-no-card" };
    if (b.length >= 3) { out.mount_fr = b[1]; out.disk_init = b[2]; }
    if (b.length >= 23) out.sector0 = probe(b, 3);
    if (b.length >= 31) { out.pt_types = [...b.subarray(23, 27)].map(x => "0x" + x.toString(16).padStart(2, "0")); out.pt1_lba = b.readUInt32LE(27); }
    if (b.length >= 51) out.part1 = probe(b, 31);
    return out;
  }
  if (status !== 0) throw new Error(`device status: ${STATUS[status] ?? status}`);
  const flags = b[3];
  return { status: "ok", mounted: b[1] === 1, fs_type: ({ 2: "FAT16", 3: "FAT32" })[b[2]] ?? `unknown(${b[2]})`,
           total_kib: b.readUInt32LE(4), free_kib: flags & 1 ? b.readUInt32LE(8) : null,
           firmware_busy: !!(flags & 2), max_name: b.readUInt16LE(12) };
}
function list(link, path) {
  const out = []; let cookie = 0;
  for (;;) {
    const b = link.request(CMD.list, Buffer.concat([u32(cookie), str(path)]), 10000);
    const next = b.readUInt32LE(1), count = b[5];
    let at = 6;
    for (let i = 0; i < count; i++) {
      const flags = b[at], size = b.readUInt32LE(at + 1), mtime = b.readUInt32LE(at + 5); at += 9;
      let name; [name, at] = readStr(b, at);
      out.push({ name, dir: !!(flags & 1), ro: !!(flags & 2), locked: !!(flags & 4), size, mtime });
    }
    if (next === 0xffffffff) break;
    cookie = next;
  }
  return out;
}
function stat(link, path) {
  const b = link.request(CMD.stat, str(path), 10000);
  return { flags: b[1], dir: !!(b[1] & 1), size: b.readUInt32LE(2), mtime: b.readUInt32LE(6) };
}
function put(link, local, remote, overwrite) {
  const data = readFileSync(local);
  const flags = (overwrite ? 1 : 0) | 2; /* create missing parents */
  const ob = link.request(CMD.openWrite, Buffer.concat([Buffer.from([flags]), u32(data.length), str(remote)]), 10000);
  let off = ob.readUInt32LE(1);
  if (off !== 0) throw new Error(`firmware offered resume at ${off}; not implemented, delete the .part files`);
  const chunk = link.maxBody - 4, t0 = Date.now();
  while (off < data.length) {
    const n = Math.min(chunk, data.length - off);
    const b = link.request(CMD.write, Buffer.concat([u32(off), data.subarray(off, off + n)]), 5000);
    const got = b.readUInt32LE(1);
    off += n;
    if (got !== off) throw new Error(`staged ${got} bytes, expected ${off}`);
    if ((off / chunk) % 64 === 0 || off === data.length)
      process.stderr.write(`\r${remote}: ${off}/${data.length} bytes (${((off / data.length) * 100).toFixed(0)}%)`);
  }
  link.request(CMD.commit, u32(crc32(data)), 10000);
  process.stderr.write(`\n${remote}: committed, crc32 ${crc32(data).toString(16)}, ${((Date.now() - t0) / 1000).toFixed(1)} s\n`);
}
function get(link, remote, local) {
  const ob = link.request(CMD.openRead, str(remote), 10000);
  const size = ob.readUInt32LE(1), parts = [];
  let off = 0;
  while (off < size) {
    const req = Buffer.alloc(6); req.writeUInt32LE(off, 0); req.writeUInt16LE(link.maxBody - 7, 4);
    const b = link.request(CMD.read, req, 5000);
    const n = b.readUInt16LE(5);
    if (n === 0) break;
    parts.push(Buffer.from(b.subarray(7, 7 + n))); off += n;
  }
  link.request(CMD.close);
  writeFileSync(local, Buffer.concat(parts));
  return off;
}

function usage() {
  console.error(`usage: hostlink-fs.mjs [-p port] <info | ls <path> | stat <path> | mkdir <path> | put <local> <remote> [--overwrite] | get <remote> <local> | rm <path>>`);
}
function main() {
  const argv = process.argv.slice(2);
  let port = null; const rest = []; let overwrite = false;
  for (let i = 0; i < argv.length; i++) {
    if (argv[i] === "-p" || argv[i] === "--port") port = argv[++i];
    else if (argv[i] === "--overwrite") overwrite = true;
    else rest.push(argv[i]);
  }
  const [cmd, a, b] = rest;
  if (!cmd || cmd === "help") { usage(); process.exit(cmd ? 0 : 1); }
  const link = new SerialLink(port ?? defaultPort());
  try {
    const h = hello(link);
    console.error(`${h.name} ${h.fw} on ${link.port}, max body ${h.maxBody}`);
    switch (cmd) {
      case "info": console.log(JSON.stringify(info(link), null, 2)); break;
      case "ls": for (const e of list(link, a ?? "/")) console.log(`${e.dir ? "d" : "-"}${e.ro ? "r" : "-"}${e.locked ? "L" : "-"}  ${String(e.size).padStart(10)}  ${e.name}`); break;
      case "stat": console.log(JSON.stringify(stat(link, a), null, 2)); break;
      case "mkdir": link.request(CMD.mkdir, str(a), 10000); console.error(`mkdir ${a}: ok`); break;
      case "put": put(link, a, b, overwrite); break;
      case "get": console.error(`got ${get(link, a, b)} bytes`); break;
      case "rm": link.request(CMD.del, str(a), 10000); console.error(`rm ${a}: ok`); break;
      default: usage(); process.exit(1);
    }
  } finally { link.close(); }
}
main();
