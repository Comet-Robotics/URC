# System Architecture

## Component Hierarchy

```
App (app/page.tsx)
├── Home (pages/homepage.tsx)
│   ├── Nav
│   ├── Video Streams Section
│   │   ├── HyperspectralVideoStream (Camera 1)
│   │   │   ├── HTML5 Video / Canvas / WebSocket Handler
│   │   │   └── Status Indicator + Stream ID
│   │   ├── HyperspectralVideoStream (Camera 2)
│   │   │   └── Same structure
│   │   └── HyperspectralVideoStream (Camera 3)
│   │       └── Same structure
│   └── Footer
├── Science (pages/science.tsx) @ /science
│   ├── Nav
│   ├── Hyperspectral Camera Stream
│   │   └── HyperspectralVideoStream (Full size)
│   └── Footer
└── Gyroscope (pages/gyroscope.tsx) @ /gyroscope
    ├── Nav
    ├── [Gyroscope content]
    └── Footer
```

## Data Flow

```
Environment Variables (.env.local)
         ↓
NEXT_PUBLIC_CAMERA_1_URL, etc.
         ↓
Homepage Component
         ↓
HyperspectralVideoStream Props
         ↓
Stream Type Detection
         ↓
         ├── MJPEG HTTP → Canvas Rendering (Image + requestAnimationFrame)
         ├── WebSocket → Canvas Rendering (Base64 decode)
         └── HTML5 → Video Element
         ↓
Display with Status Indicator & Stream ID
```

## Stream Connection Lifecycle

```
Component Mount
     ↓
Check if isMounted = true (prevents leaked state on unmount)
     ↓
Determine Stream Type from URL
     ↓
     ├── If WebSocket:
     │   ├── Open connection
     │   ├── Listen for messages (base64 frames)
     │   ├── Set isConnected = true
     │   └── On error/close → Set isConnected = false → Retry in 3s
     │
     ├── If MJPEG:
     │   ├── Start requestAnimationFrame loop
     │   ├── Fetch frame with cache-busting timestamp
     │   ├── Draw to canvas
     │   ├── Set isConnected = true
     │   └── Continue loop
     │
     └── If HTML5 Video:
         ├── Set src attribute
         ├── On loadedmetadata → Set isConnected = true
         ├── Call play() if autoplay enabled
         └── On error → Set isConnected = false

Subscribe to Updates
     ↓
Every 30 frames (for MJPEG) or message (for WebSocket)
     ↓
Update Canvas/Video Display
     ↓
User sees real-time stream

Component Unmount
     ↓
isConnected = false (cleanup)
     ↓
Close WebSocket / Cancel AnimationFrame / Pause Video
     ↓
Memory Cleaned Up
```

## File Structure

```
2025-tester-ui/
├── app/
│   ├── components/
│   │   ├── hyperspectralVideoStream.tsx ✨ NEW COMPONENT
│   │   ├── nav.tsx (UPDATED - Next.js Links)
│   │   ├── footer.tsx
│   │   ├── header.tsx
│   │   ├── notifications.tsx
│   │   ├── utilities.tsx
│   │   ├── videoStream.tsx
│   │   ├── batteryPopup.tsx
│   │   └── wifiPopup.tsx
│   ├── pages/
│   │   ├── homepage.tsx (UPDATED - Video Streams)
│   │   ├── science.tsx (UPDATED - Hyperspectral)
│   │   └── gyroscope.tsx
│   ├── science/ ✨ NEW ROUTE
│   │   └── page.tsx
│   ├── gyroscope/ ✨ NEW ROUTE
│   │   └── page.tsx
│   ├── layout.tsx
│   ├── page.tsx (UPDATED - Simplified)
│   └── globals.css (UPDATED - Removed placeholders)
├── public/
├── .env.example ✨ NEW ENV TEMPLATE
├── IMPLEMENTATION_SUMMARY.md ✨ NEW DOCS
├── QUICKSTART.md ✨ NEW GUIDE
├── VIDEO_STREAM_IMPLEMENTATION.md ✨ NEW DETAILED DOCS
├── package.json
├── next.config.ts
├── tsconfig.json
├── eslint.config.mjs
└── postcss.config.mjs
```

## Stream Type Decision Tree

```
                    HyperspectralVideoStream
                            |
                    streamUrl provided?
                    /           \
                   Yes           No (undefined)
                   |             |
                   |         Show Placeholder
                   |     "Waiting for camera..."
                   |
            Check URL pattern
            /      |       \
    ws/wss?    /stream?   Other
      |          |         |
    WebSocket  MJPEG    HTML5 Video
      |          |         |
      |      Canvas +   <video>
      |      Image      element
      |      element
      |
    WebSocket Stream Handler
    - Connect to ws://...
    - Expect base64 JPEG in messages
    - Draw to canvas
    - Auto-reconnect on failure
```

## Environment Variable Flow

```
.env.local (developer's machine)
    ↓
NEXT_PUBLIC_CAMERA_1_URL=http://localhost:8000/stream
    ↓
Next.js replaces process.env.NEXT_PUBLIC_* with values
    ↓
Homepage receives in CAMERA_STREAMS object
    ↓
Passed to HyperspectralVideoStream as streamUrl prop
    ↓
Component detects type and connects
    ↓
Video displays in UI
```

## Component Communication

```
Homepage.tsx
    |
    ├─── pass streamUrl → HyperspectralVideoStream 1
    |         |
    |         ├─ Detect type
    |         ├─ Connect to stream
    |         └─ Render video with status
    |
    ├─── pass streamUrl → HyperspectralVideoStream 2
    |         (same flow)
    |
    └─── pass streamUrl → HyperspectralVideoStream 3
              (same flow)

Science.tsx
    |
    └─── pass streamUrl → HyperspectralVideoStream
              (same flow)
```

## Browser APIs Used

```
HyperspectralVideoStream
    |
    ├── WebSocket API (for ws:// streams)
    │   ├── new WebSocket(url)
    │   ├── ws.onopen, ws.onmessage, ws.onerror, ws.onclose
    │   └── ws.close()
    |
    ├── Canvas API (for MJPEG and WebSocket)
    │   ├── canvas.getContext('2d')
    │   ├── ctx.drawImage()
    │   └── canvas width/height properties
    |
    ├── Image API (for MJPEG frame loading)
    │   ├── new Image()
    │   ├── img.src = url + timestamp
    │   ├── img.onload, img.onerror
    │   └── drawImage(img)
    |
    ├── requestAnimationFrame (for MJPEG loop)
    │   └── requestAnimationFrame(updateImage)
    |
    ├── HTML5 Video Element (for standard video)
    │   ├── <video> ref
    │   ├── video.src, video.play()
    │   ├── video.onloadedmetadata, video.onerror
    │   └── video.pause()
    |
    └── React Hooks
        ├── useState (for connection state)
        ├── useRef (for DOM elements)
        └── useEffect (for setup/cleanup)
```

## Error Handling Flow

```
Connection Error Detected
    |
    ├─ WebSocket error → setError("WebSocket connection error")
    ├─ Image load error → setError("Failed to load stream frame")
    ├─ Video error → setError("Failed to load video stream")
    └─ Auto-play error → setError("Auto-play failed: ...")
    |
    ↓
    |
Component Renders:
    ├─ error && <div> "Connection Error" + error message
    └─ Status dot turns RED
```

## Performance Considerations

- **MJPEG**: ~30 FPS via requestAnimationFrame, browser-controlled
- **WebSocket**: Frame rate depends on server sending rate
- **HTML5 Video**: Native browser handling, optimized decoder
- **Canvas**: Off-screen rendering, no reflow/repaint
- **Memory**: Auto-cleanup on component unmount via useEffect return
- **No Memory Leaks**: All intervals/animations cancelled on unmount
