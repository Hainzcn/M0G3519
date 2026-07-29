const feed = document.getElementById("live-feed");
const viewer = document.querySelector(".viewer");
const connection = document.getElementById("connection");
const connectionText = document.getElementById("connection-text");
const frameNote = document.getElementById("frame-note");
const recordButton = document.getElementById("record-button");
const recordingStatus = document.getElementById("recording-status");
const recordingCanvas = document.getElementById("recording-canvas");
const recordingsList = document.getElementById("recordings-list");
const refreshButton = document.getElementById("refresh-button");
const playback = document.getElementById("playback");
const playbackTitle = document.getElementById("playback-title");

let recorder = null;
let chunks = [];
let drawTimer = null;
let recordingStartedAt = null;
let lastFrameUrl = null;

function formatBytes(value) {
  if (value < 1024 * 1024) return `${Math.max(1, Math.round(value / 1024))} KB`;
  return `${(value / 1024 / 1024).toFixed(1)} MB`;
}

function updateConnection(status) {
  const connected = status.connected;
  connection.classList.toggle("connected", connected);
  connection.classList.toggle("error", !connected);
  connectionText.textContent = connected ? "已连接 MaixCAM2" : "等待 MaixCAM2 图像";
  frameNote.textContent = connected ? `最近一帧 ${status.last_frame_age} 秒前` : "尚未收到图像";
}

async function pollStatus() {
  try {
    const response = await fetch("/api/status", { cache: "no-store" });
    updateConnection(await response.json());
  } catch {
    connection.classList.add("error");
    connectionText.textContent = "接收服务不可用";
    frameNote.textContent = "请检查接收端程序是否启动";
  }
}

async function pullFrame() {
  try {
    const response = await fetch(`/api/frame.jpg?t=${Date.now()}`, { cache: "no-store" });
    if (response.status === 200) {
      const nextUrl = URL.createObjectURL(await response.blob());
      feed.src = nextUrl;
      if (lastFrameUrl) URL.revokeObjectURL(lastFrameUrl);
      lastFrameUrl = nextUrl;
    }
  } catch {
    // The connection status handler shows the user-facing error state.
  } finally {
    setTimeout(pullFrame, 80);
  }
}

async function loadRecordings() {
  const response = await fetch("/api/recordings", { cache: "no-store" });
  const entries = await response.json();
  recordingsList.replaceChildren();
  if (!entries.length) {
    const empty = document.createElement("p");
    empty.className = "empty-recordings";
    empty.textContent = "尚未保存录像";
    recordingsList.append(empty);
    return;
  }
  entries.forEach((entry) => {
    const item = document.createElement("button");
    item.className = "recording-item";
    item.type = "button";
    const name = document.createElement("span");
    name.className = "recording-name";
    name.textContent = entry.name.replace("recording_", "录像 ").replace(".webm", "");
    const meta = document.createElement("span");
    meta.className = "recording-meta";
    meta.textContent = `${entry.created}  ${formatBytes(entry.size)}`;
    item.append(name, meta);
    item.addEventListener("click", () => {
      document.querySelectorAll(".recording-item.selected").forEach((node) => node.classList.remove("selected"));
      item.classList.add("selected");
      playback.src = `/recordings/${encodeURIComponent(entry.name)}`;
      playbackTitle.textContent = `${entry.created} 保存的录像`;
      playback.play().catch(() => {});
    });
    recordingsList.append(item);
  });
}

function drawFrame() {
  if (!feed.naturalWidth || !feed.naturalHeight) return;
  if (recordingCanvas.width !== feed.naturalWidth || recordingCanvas.height !== feed.naturalHeight) {
    recordingCanvas.width = feed.naturalWidth;
    recordingCanvas.height = feed.naturalHeight;
  }
  recordingCanvas.getContext("2d").drawImage(feed, 0, 0, recordingCanvas.width, recordingCanvas.height);
}

async function uploadRecording(blob) {
  const response = await fetch("/api/recordings/upload", {
    method: "POST",
    headers: { "Content-Type": "video/webm" },
    body: blob,
  });
  if (!response.ok) throw new Error("保存失败");
}

function stopRecording() {
  if (recorder && recorder.state === "recording") recorder.stop();
}

function startRecording() {
  if (!feed.naturalWidth) {
    recordingStatus.textContent = "尚未收到图像，不能开始录像";
    return;
  }
  drawFrame();
  const stream = recordingCanvas.captureStream(15);
  const mimeType = MediaRecorder.isTypeSupported("video/webm;codecs=vp9") ? "video/webm;codecs=vp9" : "video/webm";
  chunks = [];
  recorder = new MediaRecorder(stream, { mimeType, videoBitsPerSecond: 2500000 });
  recorder.addEventListener("dataavailable", (event) => {
    if (event.data.size) chunks.push(event.data);
  });
  recorder.addEventListener("stop", async () => {
    clearInterval(drawTimer);
    recordButton.disabled = true;
    recordingStatus.classList.remove("active");
    recordingStatus.textContent = "正在保存录像";
    try {
      await uploadRecording(new Blob(chunks, { type: "video/webm" }));
      recordingStatus.textContent = "录像已保存";
      await loadRecordings();
    } catch {
      recordingStatus.textContent = "录像保存失败";
    } finally {
      recordButton.disabled = false;
      recordButton.textContent = "开始录像";
      recordButton.classList.remove("recording");
      recorder = null;
    }
  });
  recorder.start(1000);
  drawTimer = setInterval(drawFrame, 1000 / 15);
  recordingStartedAt = Date.now();
  recordButton.textContent = "结束并保存";
  recordButton.classList.add("recording");
  recordingStatus.classList.add("active");
  recordingStatus.textContent = "正在录像";
}

feed.addEventListener("load", () => viewer.classList.add("has-frame"));
recordButton.addEventListener("click", () => {
  if (recorder && recorder.state === "recording") stopRecording();
  else startRecording();
});
refreshButton.addEventListener("click", () => loadRecordings().catch(() => {}));

setInterval(pollStatus, 800);
pollStatus();
pullFrame();
loadRecordings().catch(() => {});
