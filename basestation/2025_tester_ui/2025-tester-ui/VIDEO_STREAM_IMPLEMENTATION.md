# Hyperspectral Camera Video Stream Implementation

## Summary

A working video streaming solution has been implemented for the hyperspectral camera and connected to the homepage. The solution supports multiple stream types and displays live video feeds in the designated video stream divs.

## What Was Done

### 1. **Created HyperspectralVideoStream Component**
   - **File**: [app/components/hyperspectralVideoStream.tsx](app/components/hyperspectralVideoStream.tsx)
   - A flexible React component that handles multiple video stream types:
     - **MJPEG streams** (HTTP-based motion JPEG) - most common for camera systems
     - **WebSocket streams** (ws:// or wss://) - real-time binary data
     - **HTML5 video streams** (MP4, WebM, etc.) - standard video formats
   
   - Features:
     - Automatic stream type detection based on URL
     - Connection status indicator (green/red dot)
     - Error handling with user-friendly messages
     - Auto-reconnection for WebSocket streams (3-second retry)
     - Stream ID label display
     - Responsive sizing using canvas/video elements

### 2. **Updated Homepage**
   - **File**: [app/pages/homepage.tsx](app/pages/homepage.tsx)
   - Replaced placeholder divs with three `HyperspectralVideoStream` components
   - Configured stream URLs via environment variables:
     - `NEXT_PUBLIC_CAMERA_1_URL` - Main camera stream
     - `NEXT_PUBLIC_CAMERA_2_URL` - Secondary camera (supports WebSocket)
     - `NEXT_PUBLIC_CAMERA_3_URL` - Tertiary camera

### 3. **Updated Science Page**
   - **File**: [app/pages/science.tsx](app/pages/science.tsx)
   - Integrated hyperspectral camera stream display
   - Configured via `NEXT_PUBLIC_HYPERSPECTRAL_URL` environment variable

### 4. **Updated CSS Styling**
   - **File**: [app/globals.css](app/globals.css)
   - Removed placeholder background images from `.video-stream-div` and `.hyperspec` classes
   - Added `.hyperspectral-video-container` class for proper video display
   - Maintained responsive sizing and styling

### 5. **Fixed Navigation & Routing**
   - **File**: [app/components/nav.tsx](app/components/nav.tsx)
   - Replaced React Router `NavLink` with Next.js `Link` component
   - Updated to use Next.js file-based routing
   - Routes: `/`, `/science`, `/gyroscope`

### 6. **Set Up Next.js File-Based Routing**
   - Created route handlers:
     - [app/science/page.tsx](app/science/page.tsx) - `/science` route
     - [app/gyroscope/page.tsx](app/gyroscope/page.tsx) - `/gyroscope` route
   - Updated [app/page.tsx](app/page.tsx) - `/` home route

## Configuration

### Environment Variables
Create a `.env.local` file (or copy from `.env.example`) and update the camera stream URLs:

```env
NEXT_PUBLIC_CAMERA_1_URL=http://localhost:8000/stream
NEXT_PUBLIC_CAMERA_2_URL=ws://localhost:8000/stream2
NEXT_PUBLIC_CAMERA_3_URL=http://localhost:8000/stream3
NEXT_PUBLIC_HYPERSPECTRAL_URL=http://localhost:8000/hyperspectral/stream
```

### URL Format Guidelines

- **MJPEG HTTP streams**: `http://camera-ip:port/stream`
- **WebSocket streams**: `ws://camera-ip:port/stream` or `wss://camera-ip:port/stream` (secure)
- **Standard video**: `http://camera-ip:port/video.mp4`

## How It Works

### Stream Detection Logic
The component automatically detects and handles different stream types:

```
If URL starts with "ws://" or "wss://" → WebSocket stream
Else if URL contains "/stream" or "mjpeg" → MJPEG stream (canvas-based)
Else → HTML5 video element (mp4, webm, etc.)
```

### MJPEG Stream Handling
- Uses an Image element with continuous timestamp updating to bypass caching
- Draws frames to a canvas at ~30 FPS via `requestAnimationFrame`
- Handles reconnection automatically

### WebSocket Stream Handling
- Expects base64-encoded JPEG data in messages
- Decodes and displays frames on canvas
- Auto-reconnects after 3 seconds on disconnect

### Component Props
```typescript
interface HyperspectralVideoStreamProps {
  streamId: string;        // Display label (e.g., "Camera 1 - Main Stream")
  streamUrl?: string;      // Stream URL (optional - shows placeholder if omitted)
  width?: number;          // Canvas width (default: 640)
  height?: number;         // Canvas height (default: 480)
  autoplay?: boolean;      // Auto-play video (default: true)
}
```

## Testing the Implementation

### Development Server
```bash
npm run dev
```
Opens at `http://localhost:3000`

### Production Build
```bash
npm run build
npm start
```

### Features Visible in UI
- ✅ Three video stream divs on homepage (replacing placeholder images)
- ✅ Hyperspectral camera stream on science page
- ✅ Green status indicator when stream is connected
- ✅ Red status indicator when stream fails
- ✅ Stream ID labels on each video
- ✅ Error messages when connection fails
- ✅ "Waiting for camera connection..." placeholder when no URL provided

## Backend Requirements

Your camera/hyperspectral system should provide one of these stream types:

### Option 1: MJPEG HTTP Stream (Recommended)
- Endpoint: `GET /stream` or `/mjpeg`
- Returns multipart/x-mixed-replace with JPEG frames
- Content-Type: `multipart/x-mixed-replace; boundary=--boundary`
- Simplest to implement, widely supported

### Option 2: WebSocket Stream
- Endpoint: `ws://device/stream`
- Send base64-encoded JPEG frames as text messages
- Allows real-time control, lower latency potential

### Option 3: Standard HTML5 Video
- Endpoint: `GET /video.mp4` or similar
- Returns MP4, WebM, or other video format
- Standard browser video support

## File Structure
```
app/
├── components/
│   ├── hyperspectralVideoStream.tsx  (NEW)
│   ├── nav.tsx                        (UPDATED)
│   ├── footer.tsx
│   ├── header.tsx
│   ├── notifications.tsx
│   ├── utilities.tsx
│   ├── videoStream.tsx
│   ├── batteryPopup.tsx
│   └── wifiPopup.tsx
├── pages/
│   ├── homepage.tsx                   (UPDATED)
│   ├── science.tsx                    (UPDATED)
│   └── gyroscope.tsx
├── globals.css                        (UPDATED)
├── layout.tsx
├── page.tsx                           (UPDATED)
├── science/page.tsx                   (NEW)
└── gyroscope/page.tsx                 (NEW)
```

## Troubleshooting

### No video appears
1. Check browser console for errors
2. Verify camera backend is running and accessible
3. Check environment variables are set correctly
4. Test URL with curl: `curl -v http://camera-ip:port/stream`

### "Connection Error" message
- WebSocket: Ensure `ws://` or `wss://` protocol is correct
- MJPEG: Check HTTP endpoint is accessible
- CORS: Camera backend may need CORS headers if streaming from different domain

### Status indicator stays red
- Network connectivity issue
- Camera backend is down
- Incorrect URL in environment variable
- Port is blocked by firewall

## Next Steps

1. **Update environment variables** with your actual camera endpoints
2. **Test connectivity** to camera backends
3. **Adjust canvas dimensions** if needed (width/height props)
4. **Implement backend stream server** if not already available
5. **Add CI/CD configuration** for deployment

## Dependencies Used
- React 19.2.0
- Next.js 16.0.3
- TypeScript 5.9.3
- Tailwind CSS 4 (via postcss)

No additional npm packages were needed - the solution uses only browser APIs (WebSocket, Canvas, Image, Video elements).
