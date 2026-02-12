'use client';

import React, { useEffect, useRef, useState } from 'react';

interface HyperspectralVideoStreamProps {
  streamId: string;
  streamUrl?: string;
  width?: number;
  height?: number;
  autoplay?: boolean;
}

const HyperspectralVideoStream: React.FC<HyperspectralVideoStreamProps> = ({
  streamId,
  streamUrl,
  width = 640,
  height = 480,
  autoplay = true,
}) => {
  const videoRef = useRef<HTMLVideoElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [isConnected, setIsConnected] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [isMounted, setIsMounted] = useState(false);
  const wsRef = useRef<WebSocket | null>(null);
  const animationFrameRef = useRef<number | null>(null);

  // Initialize mount state
  useEffect(() => {
    setIsMounted(true);
    return () => setIsMounted(false);
  }, []);

  // Handle MJPEG streams (most common for camera systems)
  const connectMJPEGStream = (url: string) => {
    if (!isMounted) return;

    const img = new Image();
    img.crossOrigin = 'anonymous';
    
    const updateImage = () => {
      if (!isMounted) return;
      const canvas = canvasRef.current;
      if (!canvas) {
        animationFrameRef.current = requestAnimationFrame(updateImage);
        return;
      }
      
      const ctx = canvas.getContext('2d');
      if (!ctx) {
        animationFrameRef.current = requestAnimationFrame(updateImage);
        return;
      }

      // Add timestamp to bypass cache
      const timestamp = Date.now();
      img.src = `${url}?t=${timestamp}`;
      
      img.onload = () => {
        if (isMounted && canvas) {
          ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
        }
        if (isMounted) {
          animationFrameRef.current = requestAnimationFrame(updateImage);
        }
      };

      img.onerror = () => {
        if (isMounted) {
          setError('Failed to load stream frame');
          animationFrameRef.current = requestAnimationFrame(updateImage);
        }
      };
    };

    updateImage();
    setIsConnected(true);
    setError(null);
  };

  // Handle WebSocket streams
  const connectWebSocketStream = (url: string) => {
    if (!isMounted) return;

    try {
      const ws = new WebSocket(url);
      wsRef.current = ws;

      ws.onopen = () => {
        if (isMounted) {
          setIsConnected(true);
          setError(null);
          console.log(`WebSocket connected: ${streamId}`);
        }
      };

      ws.onmessage = (event) => {
        if (!isMounted) return;
        
        const canvas = canvasRef.current;
        if (!canvas) return;

        const ctx = canvas.getContext('2d');
        if (!ctx) return;

        // Assume the message contains base64 encoded image data
        if (typeof event.data === 'string') {
          const img = new Image();
          img.crossOrigin = 'anonymous';
          img.onload = () => {
            if (isMounted && canvas && ctx) {
              ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
            }
          };
          img.src = `data:image/jpeg;base64,${event.data}`;
        }
      };

      ws.onerror = () => {
        if (isMounted) {
          setError('WebSocket connection error');
          setIsConnected(false);
        }
      };

      ws.onclose = () => {
        if (isMounted) {
          setIsConnected(false);
          // Attempt to reconnect after 3 seconds
          setTimeout(() => {
            if (isMounted) {
              connectWebSocketStream(url);
            }
          }, 3000);
        }
      };
    } catch (err) {
      if (isMounted) {
        setError(`WebSocket connection failed: ${err}`);
        setIsConnected(false);
      }
    }
  };

  // Handle HTML5 video streams
  const connectHTMLVideoStream = (url: string) => {
    if (!isMounted) return;
    
    const videoElement = videoRef.current;
    if (!videoElement) return;

    videoElement.src = url;
    videoElement.onloadedmetadata = () => {
      if (isMounted) {
        setIsConnected(true);
        setError(null);
      }
    };

    videoElement.onerror = () => {
      if (isMounted) {
        setError('Failed to load video stream');
        setIsConnected(false);
      }
    };

    if (autoplay) {
      videoElement.play().catch((err) => {
        if (isMounted) {
          setError(`Auto-play failed: ${err.message}`);
        }
      });
    }
  };

  useEffect(() => {
    if (!isMounted) return;

    // Determine the stream type and connect accordingly
    if (streamUrl) {
      if (streamUrl.startsWith('ws://') || streamUrl.startsWith('wss://')) {
        connectWebSocketStream(streamUrl);
      } else if (streamUrl.includes('/stream') || streamUrl.includes('mjpeg')) {
        // MJPEG stream
        connectMJPEGStream(streamUrl);
      } else {
        // Try HTML5 video first
        connectHTMLVideoStream(streamUrl);
      }
    }

    return () => {
      // Cleanup
      if (wsRef.current) {
        wsRef.current.close();
        wsRef.current = null;
      }
      if (animationFrameRef.current) {
        cancelAnimationFrame(animationFrameRef.current);
        animationFrameRef.current = null;
      }
      if (videoRef.current) {
        videoRef.current.pause();
        videoRef.current.src = '';
      }
    };
  }, [streamUrl, autoplay, isMounted]);

  if (!isMounted) {
    return <div className="w-full h-full bg-black" />;
  }

  return (
    <div className="hyperspectral-video-container relative w-full h-full bg-black rounded-lg overflow-hidden">
      {/* Video element for HTML5 video streams */}
      {!streamUrl?.includes('/stream') && !streamUrl?.includes('mjpeg') && !streamUrl?.startsWith('ws') && (
        <video
          ref={videoRef}
          className="w-full h-full object-contain"
          autoPlay={autoplay}
          controls={false}
          muted
        />
      )}

      {/* Canvas for MJPEG and WebSocket streams */}
      {(streamUrl?.includes('/stream') || streamUrl?.includes('mjpeg') || streamUrl?.startsWith('ws')) && (
        <canvas
          ref={canvasRef}
          width={width}
          height={height}
          className="w-full h-full object-contain"
        />
      )}

      {/* Placeholder when no URL is provided */}
      {!streamUrl && (
        <div className="absolute inset-0 flex items-center justify-center bg-gray-900/80 text-white">
          <div className="text-center">
            <div className="text-lg font-semibold mb-2">No Stream Available</div>
            <div className="text-sm text-gray-400">Waiting for camera connection...</div>
          </div>
        </div>
      )}

      {/* Error state */}
      {error && (
        <div className="absolute inset-0 flex items-center justify-center bg-blue-900/80 text-white">
          <div className="text-center">
            <div className="text-lg font-semibold mb-2">Connection Error</div>
            <div className="text-sm text-blue-200">{error}</div>
          </div>
        </div>
      )}

      {/* Status indicator */}
      {streamUrl && (
        <div className="absolute top-2 right-2 flex items-center gap-2 bg-black/70 px-3 py-2 rounded-lg">
          <div className={`w-2 h-2 rounded-full ${isConnected ? 'bg-green-500' : 'bg-red-500'}`} />
          <span className="text-xs text-white font-medium">
            {isConnected ? 'Connected' : 'Disconnected'}
          </span>
        </div>
      )}

      {/* Stream ID label */}
      <div className="absolute bottom-2 left-2 bg-black/70 text-white px-3 py-2 rounded-lg text-xs font-semibold">
        {streamId}
      </div>
    </div>
  );
};

export default HyperspectralVideoStream;
