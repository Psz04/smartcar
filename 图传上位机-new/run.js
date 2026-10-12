const WebSocket = require('ws');
const fs = require('fs');
const http = require('http');
const path = require('path');

let img_idx = 0;
let carSocket = null;
let carConnected = false;
let imageStorageEnabled = false;

// 3000 端口：车端 WebSocket 连接。
const wssReceive = new WebSocket.Server({ port: 3000 });
console.log('WebSocket server started on port 3000 (car)');

// 3001 端口：网页浏览器连接。
const wssSend = new WebSocket.Server({ port: 3001 });
console.log('WebSocket server started on port 3001 (browser)');
console.log('Image storage default: off');

function ensureDirectoryExists(dirPath) {
    if (!fs.existsSync(dirPath)) {
        fs.mkdirSync(dirPath, { recursive: true });
    }
}

ensureDirectoryExists('./image');
ensureDirectoryExists('./data');

// 解析车端二进制图像帧：
// [1B TYPE=0x01][4B meta length][meta text][JPEG data]
function parseVehicleImageFrame(message) {
    if (!message || message.length < 5) {
        return null;
    }
    const frameType = message.readUInt8(0);
    if (frameType !== 0x01) {
        return null;
    }
    const metaLen = message.readUInt32BE(1);
    if (message.length < 5 + metaLen) {
        return null;
    }
    const metaText = message.slice(5, 5 + metaLen).toString('utf8');
    const imageData = message.slice(5 + metaLen);
    return { metaText, imageData };
}

function parseMetaText(metaText) {
    const params = {};
    if (!metaText) {
        return params;
    }
    metaText.split(';').forEach((item) => {
        const separator = item.indexOf('=');
        if (separator > 0) {
            const key = item.substring(0, separator).trim();
            const value = item.substring(separator + 1).trim();
            params[key] = value;
        }
    });
    return params;
}

function saveParamsToFile(params, filePath) {
    try {
        let content = '';
        for (const [name, value] of Object.entries(params)) {
            content += `${name}: ${value}\n`;
        }
        fs.writeFileSync(filePath, content, 'utf8');
        return true;
    } catch (error) {
        console.error('save params failed:', error);
        return false;
    }
}

function broadcastToBrowsers(payload) {
    const text = JSON.stringify(payload);
    wssSend.clients.forEach((client) => {
        if (client.readyState === WebSocket.OPEN) {
            client.send(text);
        }
    });
}

// 车端连接 3000 端口。
wssReceive.on('connection', (ws) => {
    carSocket = ws;
    carConnected = true;
    console.log('car connected to port 3000');

    ws.on('message', (message, isBinary) => {
        if (isBinary) {
            const frame = parseVehicleImageFrame(message);
            if (!frame) {
                console.error('invalid vehicle image frame');
                return;
            }

            const params = parseMetaText(frame.metaText);
            const currentIndex = img_idx;
            const txtFilePath = `./data/${currentIndex}.txt`;
            const imageFilePath = `./image/${currentIndex}.jpg`;
            img_idx++;

            // 只有打开图像存储开关时才写入文件。
            if (imageStorageEnabled === true) {
                saveParamsToFile(params, txtFilePath);
                if (frame.imageData.length > 0) {
                    fs.writeFile(imageFilePath, frame.imageData, (err) => {
                        if (err) {
                            console.error('save image failed:', err);
                        }
                    });
                }
            }

            // 浏览器始终显示图像数据，不受存储开关影响。
            broadcastToBrowsers({
                type: 'new_image',
                image: frame.imageData.length > 0 ? frame.imageData : null,
                idx: currentIndex,
                params,
                saved: (imageStorageEnabled === true),
            });
        } else {
            const text = message.toString('utf8');
            console.log('car text:', text);
            broadcastToBrowsers({ type: 'vehicle_message', message: text });
        }
    });

    ws.on('close', () => {
        carConnected = false;
        carSocket = null;
        console.log('car disconnected');
    });
});

// 网页连接 3001 端口。
wssSend.on('connection', (ws) => {
    console.log('browser connected to port 3001');

    // 新网页连接时同步当前图像存储开关状态。
    ws.send(JSON.stringify({
        type: 'image_storage_state',
        enabled: imageStorageEnabled,
    }));

    ws.on('message', (message) => {
        let data = null;
        try {
            data = JSON.parse(message.toString());
        } catch (error) {
            ws.send(JSON.stringify({
                type: 'status_info',
                message: 'JSON parse failed',
            }));
            return;
        }

        if (data.type === 'command') {
            const command = String(data.data || '').trim();
            if (!command) {
                ws.send(JSON.stringify({
                    type: 'status_info',
                    message: 'empty command',
                }));
                return;
            }
            if (carSocket && carSocket.readyState === WebSocket.OPEN) {
                carSocket.send(command);
                ws.send(JSON.stringify({ type: 'command_ack', data: command }));
            } else {
                ws.send(JSON.stringify({
                    type: 'status_info',
                    message: 'car is not connected',
                }));
            }
            return;
        }

        if (data.type === 'heartbeat') {
            ws.send(JSON.stringify({
                type: 'heartbeat_response',
                status: carConnected,
                timestamp: data.timestamp,
            }));
            return;
        }

        if (data.type === 'set_image_storage') {
            imageStorageEnabled = (data.enabled === true);
            console.log('image storage:', imageStorageEnabled ? 'on' : 'off');
            broadcastToBrowsers({
                type: 'image_storage_state',
                enabled: imageStorageEnabled,
            });
            return;
        }

        // 兼容旧版发车/帧率命令；新版网页统一走 command。
        if (data.type === 'start') {
            if (carSocket && carSocket.readyState === WebSocket.OPEN) {
                carSocket.send(data.frameRate === 0 ? 'STOP' : 'START');
            }
            return;
        }
        if (data.type === 'set_fps') {
            if (carSocket && carSocket.readyState === WebSocket.OPEN) {
                carSocket.send(`SET_FPS,${data.value}`);
            }
            return;
        }

        ws.send(JSON.stringify({
            type: 'status_info',
            message: `unknown browser message: ${data.type}`,
        }));
    });

    ws.on('close', () => {
        console.log('browser disconnected');
    });
});

const server = http.createServer((req, res) => {
    if (req.url === '/' || req.url === '/index.html') {
        const filePath = path.join(__dirname, 'web.html');
        fs.readFile(filePath, (err, data) => {
            if (err) {
                res.writeHead(500);
                res.end('Error loading web.html');
            } else {
                res.writeHead(200, { 'Content-Type': 'text/html; charset=utf-8' });
                res.end(data);
            }
        });
    } else {
        res.writeHead(404);
        res.end('Not Found');
    }
});

server.listen(8080, () => {
    console.log('HTTP server started on port 8080');
});