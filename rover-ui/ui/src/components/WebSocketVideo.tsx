import { useEffect, useState, useRef } from "react";
import {Card} from "@/components/ui/card.tsx";
import JMuxer from "jmuxer"

const WebSocketVideo = ({ wsUrl = "/ws", width = 640, height = 480 }) => {
    const videoRef = useRef<HTMLVideoElement | null>(null);

  
    useEffect(() => {

        let jmuxer = new JMuxer({
            node: videoRef.current!,
            mode: 'video',
            flushingTime: 1000,
            fps: 30,
            debug: true,
            onError: function(data) {
                if (/Safari/.test(navigator.userAgent) && /Apple Computer/.test(navigator.vendor)) {
                    jmuxer.reset();
                }
            }
        });


      // WebSocket connection
      const ws = new WebSocket(wsUrl);
      ws.binaryType = 'arraybuffer';

      ws.onmessage = (event) => {
        const data = new Uint8Array(event.data);
      jmuxer.feed({
          video: data
      });
      };

      return () => {
        ws.close();
      };
    }, [videoRef]);
  

    return (
      <div>
        <h1>H.264 Stream Player</h1>
          <video ref={videoRef} width={480} height={640}></video>
      </div>
    );
};

export default WebSocketVideo;
