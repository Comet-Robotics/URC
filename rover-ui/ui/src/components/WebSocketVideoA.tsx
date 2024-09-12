import { useEffect, useState, useRef } from "react";
import {Card} from "@/components/ui/card.tsx";

const WebSocketVideo = ({ wsUrl = "/ws", width = 640, height = 480 }) => {
    const videoRef = useRef<HTMLVideoElement | null>(null);
    const mediaSourceRef = useRef<MediaSource | null>(null);
    const sourceBufferRef = useRef<SourceBuffer | null>(null);
  
    useEffect(() => {
      // Initialize MediaSource
      const mediaSource = new MediaSource();
      mediaSourceRef.current = mediaSource;
  
      const videoElement = videoRef.current;
      videoElement!.src = URL.createObjectURL(mediaSource);
  
      mediaSource.addEventListener('sourceopen', handleSourceOpen);
  
      // WebSocket connection
      const ws = new WebSocket(wsUrl);
      ws.binaryType = 'arraybuffer';
  
      ws.onmessage = (event) => {
        const data = new Uint8Array(event.data);
        appendBuffer(data);
      };
  
      return () => {
        ws.close();
        mediaSource.removeEventListener('sourceopen', handleSourceOpen);
      };
    }, []);
  
    const handleSourceOpen = () => {
      const mediaSource = mediaSourceRef.current;
      const mimeCodec = 'video/mp4; codecs="avc1.42E01E"'; // H.264 codec inside MP4 container
  
      if (mediaSource && MediaSource.isTypeSupported(mimeCodec)) {
        console.log("MIME type or codec supported");
        // Create a SourceBuffer for the H.264 video
        const sourceBuffer = mediaSource.addSourceBuffer(mimeCodec);
        sourceBufferRef.current = sourceBuffer;
      } else {
        console.error('MIME type or codec not supported');
      }
    };
  
    const appendBuffer = (data:any) => {
        const sourceBuffer = sourceBufferRef.current;
    
        if (sourceBuffer && !sourceBuffer.updating) {
            try {
                sourceBuffer.appendBuffer(data);
            } catch (err) {
            console.error('Error appending buffer', err);
            }
        }
    };
    return (
      <div>
        <h1>H.264 Stream Player</h1>
        <video ref={videoRef} controls autoPlay style={{ width: width, height: height }} />
      </div>
    );
};

export default WebSocketVideo;
