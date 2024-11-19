import { useEffect, useRef, useState } from 'react';

import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import {
  Card,
  CardContent,
  CardHeader,
  CardTitle,
} from "@/components/ui/card"

const StartStream = () => {
  const [remoteVideos, setRemoteVideos] = useState<Map<string, HTMLVideoElement>>(new Map());
  const [selectedVideo, setSelectedVideo] = useState<string | null>(null);
  const [pc, setPc] = useState<RTCPeerConnection | null>(null);

  const videoContainerRef = useRef<HTMLDivElement | null>(null);

  const log = (message: string) => {
    console.log(message);
  };

  useEffect(() => {
    const startSession = async () => {
      console.log("Starting Session");

      const peerConnection = new RTCPeerConnection({
        iceServers: []
      });
      setPc(peerConnection);

      // Handle incoming tracks
      peerConnection.ontrack = (event) => {
        const el = document.createElement(event.track.kind) as HTMLVideoElement;
        el.srcObject = event.streams[0];
        el.autoplay = true;
        el.controls = false;
        if( selectedVideo === null) {
          setSelectedVideo(event.streams[0].id);
        }
        setRemoteVideos((prevVideos) => new Map(prevVideos).set(event.streams[0].id, el));
      };

      // Log the ICE connection state
      peerConnection.oniceconnectionstatechange = () => log(peerConnection.iceConnectionState);

      // Handle ICE candidate event
      peerConnection.onicegatheringstatechange = async () => {
        if (peerConnection.iceGatheringState === 'complete') {
          console.log("Sending local descriptor");
          const localDescriptor = btoa(JSON.stringify(peerConnection.localDescription));

          try {
            const response = await fetch('/start_stream', {
              method: 'POST',
              headers: { 'Content-Type': 'application/json' },
              body: JSON.stringify({ localSessionDescription: localDescriptor })
            });

            if (response.ok) {
              const remoteSessionDescription = await response.text();
              try {
                peerConnection.setRemoteDescription(
                  new RTCSessionDescription(JSON.parse(atob(remoteSessionDescription)))
                );
                log("Session established.");
              } catch (error) {
                console.error('Error setting remote description:', error);
                log('Error setting remote description.');
              }
            } else {
              log("Failed to start stream.");
            }
          } catch (error) {
            console.error('Error exchanging session descriptors:', error);
            log('Error exchanging session descriptors.');
          }
        }
      };

      // Set up transceivers and create an offer
      peerConnection.addTransceiver('video', { direction: 'recvonly' });
      peerConnection.addTransceiver('video', { direction: 'recvonly' });

      try {
        const offer = await peerConnection.createOffer();
        await peerConnection.setLocalDescription(offer);
      } catch (error) {
        log('Failed to create offer.');
      }
    };

    startSession();

    return () => {
      // Clean up on component unmount
      pc?.close();
      remoteVideos.forEach((video) => {
        video.srcObject = null;
        video.remove();
      });
    };
  }, []);

  useEffect(() => {
    // Clear the video container
    if (videoContainerRef.current) {
      videoContainerRef.current.innerHTML = '';
    }

    // Append the selected video to the container
    if (selectedVideo && videoContainerRef.current) {
      const videoElement = remoteVideos.get(selectedVideo);
      if (videoElement) {
        videoContainerRef.current.appendChild(videoElement);
      }
    }
  }, [selectedVideo, remoteVideos]);


  return (
    <Card className='w-full h-full' >
      <CardHeader className="flex flex-row justify-between items-center">
          <CardTitle>Streams</CardTitle>
        <Select onValueChange={(value) => setSelectedVideo(value)}>
          <SelectTrigger className="w-[180px]">
            <SelectValue placeholder="Select Video" />
          </SelectTrigger>
          <SelectContent>
            {Array.from(remoteVideos.keys()).map((id: string) => (
              <SelectItem key={id} value={id}>
                {id}
              </SelectItem>
            ))}
          </SelectContent>
        </Select>
      </CardHeader>
      <CardContent>
   
      <div id="remoteVideos"  ref={videoContainerRef} className='w-[640px] h-[480px]' />
      </CardContent>
    </Card>
  );
};

export default StartStream;
