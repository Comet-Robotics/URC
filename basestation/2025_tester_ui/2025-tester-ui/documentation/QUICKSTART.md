# Quick Start Guide - Video Stream Setup

## 1. Configure Environment Variables

```bash
# Copy the example file
cp .env.example .env.local

# Edit .env.local with your camera endpoints
nano .env.local
```

Replace the URLs with your actual camera stream endpoints:

```env
NEXT_PUBLIC_CAMERA_1_URL=http://your-camera-ip:8000/stream
NEXT_PUBLIC_CAMERA_2_URL=ws://your-camera-ip:8001/stream
NEXT_PUBLIC_CAMERA_3_URL=http://your-camera-ip:8002/stream
NEXT_PUBLIC_HYPERSPECTRAL_URL=http://your-camera-ip:8003/hyperspectral
```

## 2. Start Development Server

```bash
npm run dev
```

The app will be available at `http://localhost:3000`

## 3. Verify Video Streams Display

- Navigate to the **Home** page (/)
  - Should see three video stream containers
  - Each will show either:
    - Live video (if camera is running)
    - "Waiting for camera connection..." (if camera URL not set)
    - "Connection Error" (if URL is invalid or camera is offline)

- Navigate to **Science Payload** page (/science)
  - Should see the hyperspectral camera stream

- Check **Status Indicators**
  - Green dot = Stream connected
  - Red dot = Stream disconnected

## 4. Stream Type Support

The system automatically detects your stream type:

| URL Pattern | Type | Example |
|------------|------|---------|
| Contains `/stream` or `mjpeg` | MJPEG (HTTP) | `http://camera:8000/stream` |
| Starts with `ws://` or `wss://` | WebSocket | `ws://camera:8000/stream` |
| Other HTTP URLs | HTML5 Video | `http://camera:8000/video.mp4` |

## 5. Test with Mock Camera Stream

If you don't have a real camera yet, you can test with a mock MJPEG server:

```bash
# Install a simple HTTP server
python3 -m http.server 8000 --directory ./public

# Set environment variable to test
export NEXT_PUBLIC_CAMERA_1_URL="http://localhost:8000"
```

## 6. For Production Deployment

```bash
npm run build
npm run start
```

## 7. Troubleshooting

| Issue | Solution |
|-------|----------|
| No video appears | Check browser DevTools Console for errors |
| "Connection Error" displayed | Verify camera URL is correct and camera is running |
| Status indicator is red | Check camera is accessible from your network |
| CORS error in console | Camera backend needs CORS headers configured |

## File Locations

- **Video Stream Component**: `app/components/hyperspectralVideoStream.tsx`
- **Homepage**: `app/pages/homepage.tsx`
- **Science Page**: `app/pages/science.tsx`
- **CSS Styles**: `app/globals.css`
- **Navigation**: `app/components/nav.tsx`

## Next Steps

1. Set up your camera streaming backend (MJPEG, WebSocket, or standard video)
2. Test connectivity: `curl -v http://camera-ip:8000/stream`
3. Update `.env.local` with real URLs
4. Deploy to your rover system
