import React, { useEffect, useRef, useState, useCallback } from 'react';
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

interface RemoteVideoProps {
  streamId: string;
  stream: MediaStream;
}

const RemoteVideo = React.memo(({ streamId, stream }: RemoteVideoProps) => {
  const videoRef = useRef<HTMLVideoElement>(null);

  useEffect(() => {
    const video = videoRef.current;
    if (!video) return;

    video.srcObject = stream;
    video.autoplay = true;
    video.controls = false;
    video.width = 640;
    video.height = 480;

    return () => {
      console.log(`Cleaning Up Video ${streamId}`)
      // // Cleanup when the component unmounts or stream changes
      // if (video.srcObject) {
      //   const tracks = (video.srcObject as MediaStream).getTracks();
      //   tracks.forEach(track => track.stop());
      // }
      // video.srcObject = null;
    };
  }, [stream, streamId]);

  return (
    <div className="relative">
      <video ref={videoRef} playsInline className="w-full h-full max-h-[70vh] object-contain" />
      <div className="absolute bottom-2 left-2 bg-gray-800 text-white p-1 rounded text-sm">{streamId}</div>
    </div>
  );
});

const StartStream = () => {
  const [remoteStreams, setRemoteStreams] = useState<Map<string, MediaStream>>(new Map());
  const [selectedVideo, setSelectedVideo] = useState<string | null>("both"); // Default to single
  const [pc, setPc] = useState<RTCPeerConnection | null>(null);
  const videoContainerRef = useRef<HTMLDivElement | null>(null);
  const [isFullscreen, setIsFullscreen] = useState(false);

  const log = useCallback((message: string) => {
    console.log(message);
  }, []);

  const toggleFullscreen = useCallback(() => {
    const element = videoContainerRef.current;
    if (!element) return;

    if (!document.fullscreenElement) {
      element.requestFullscreen()
        .then(() => {
          setIsFullscreen(true);
        })
        .catch(err => {
          console.error(`Error attempting to enable fullscreen mode: ${err.message}`);
        });
    } else {
      document.exitFullscreen();
      setIsFullscreen(false);
    }
  }, []);

  useEffect(() => {
    const handleFullscreenChange = () => {
      setIsFullscreen(!!document.fullscreenElement);
    };

    document.addEventListener('fullscreenchange', handleFullscreenChange);

    return () => {
      document.removeEventListener('fullscreenchange', handleFullscreenChange);
    };
  }, []);

  useEffect(() => {
    const startSession = async () => {
      console.log("Starting Session");

      const peerConnection = new RTCPeerConnection({
        iceServers: [] // Add your ICE servers here for production
      });
      setPc(peerConnection);

      // Handle incoming tracks
      peerConnection.ontrack = (event) => {
        const stream = event.streams[0];
        const streamId = stream.id;

        setRemoteStreams((prevStreams) => {
          const newStreams = new Map(prevStreams);
          newStreams.set(streamId, stream);
          return newStreams;
        });

        //if (selectedVideo === null || selectedVideo === "single") { //select the first video if on single mode
        //  setSelectedVideo(streamId);
        //}
      };

      // Log the ICE connection state
      peerConnection.oniceconnectionstatechange = () => log(`ICE Connection State: ${peerConnection.iceConnectionState}`);

      peerConnection.onicecandidate = (event) => {
        if (event.candidate) {
          console.log('New ICE candidate:', event.candidate);
        }
      };

      // Handle ICE gathering state event
      peerConnection.onicegatheringstatechange = async () => {
        console.log("ICE gathering state changed to:", peerConnection.iceGatheringState);  // Debugging

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
                const remoteDesc = new RTCSessionDescription(JSON.parse(atob(remoteSessionDescription)));
                await peerConnection.setRemoteDescription(remoteDesc);
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
        console.error('Failed to create offer:', error); // Log the error
        log('Failed to create offer.');
      }
    };

    startSession();

    return () => {
      // Clean up on component unmount
      console.log("Cleaning up...")
      pc?.close();
      remoteStreams.forEach((stream) => {
        stream.getTracks().forEach(track => track.stop());
      });
    };
  }, [log]);

  const streamIds = Array.from(remoteStreams.keys());

  return (
    <Card className='col-span-3'>
      <CardHeader className="flex flex-row justify-between items-center">
        <CardTitle>Streams</CardTitle>
        <div className="flex items-center space-x-2">
          <Select defaultValue='both' onValueChange={(value) => setSelectedVideo(value)}>
            <SelectTrigger className="w-[180px]">
              <SelectValue placeholder="Select Video" />
            </SelectTrigger>
            <SelectContent>
  
              <SelectItem key="both" value="both">
                Both
              </SelectItem>
              {streamIds.map((id: string) => (
                <SelectItem key={id} value={id}>
                  {id}
                </SelectItem>
              ))}
            </SelectContent>
          </Select>
          <button onClick={toggleFullscreen} className="px-4 py-2 bg-blue-500 text-white rounded hover:bg-blue-700">
            {isFullscreen ? 'Exit Fullscreen' : 'Fullscreen'}
          </button>
        </div>
      </CardHeader>
      <CardContent>
        <div ref={videoContainerRef} className="w-full h-full z-50" style={{ backgroundColor: 'black' }}>
          {selectedVideo !== "both" && streamIds.length > 0 ? (
            <RemoteVideo streamId={selectedVideo!}  stream={remoteStreams.get(selectedVideo!)!} />
          ) : selectedVideo === "both" && streamIds.length >= 2 ? (
            <div className="flex w-full h-full">
              <div className="w-1/2 h-full">
                <RemoteVideo streamId={streamIds[0]} stream={remoteStreams.get(streamIds[0])!} />
              </div>
              <div className="w-1/2 h-full">
                <RemoteVideo streamId={streamIds[1]} stream={remoteStreams.get(streamIds[1])!} />
              </div>
            </div>
          ) : (
            <div className="text-white text-center py-4">
              {streamIds.length === 0 ? "No streams available" : "Select a stream or 'Both'"}
            </div>
          )}
        </div>
      </CardContent>
    </Card>
  );
};

export default StartStream;