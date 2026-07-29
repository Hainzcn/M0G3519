# MaixCAM2 图传接收端

双击 `start_receiver.bat` 启动服务。浏览器打开 `http://127.0.0.1:8000`。

MaixCAM2 与笔记本连接至同一局域网后，将每一帧 JPEG 图像以 HTTP POST 方式发送至：

```text
http://笔记本局域网地址:8000/api/frame
```

请求头设置为 `Content-Type: image/jpeg`，请求体直接放入 JPEG 二进制数据。页面收到图像后会实时显示。点击开始录像会在浏览器端录制实时画面，点击结束并保存后，视频保存为 WebM 文件，可在历史录像中回放。

比赛时将本服务运行在赛道外的笔记本电脑。视频文件保存在 `recordings` 文件夹，应在每次测试结束后确认录像已保存。
