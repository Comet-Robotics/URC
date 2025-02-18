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
} from "@/components/ui/card";
import { eventNames } from 'process';

interface RemoteVideoProps {
  streamId: string;
  stream: MediaStream;
  pc: RTCPeerConnection | null; // Pass the peer connection
}

const RemoteVideo = React.memo(({ streamId, stream, pc }: RemoteVideoProps) => {
  const videoRef = useRef<HTMLVideoElement>(null);
  const [bitrate, setBitrate] = useState<number | null>(null);
  const [fps, setFps] = useState<number | null>(null);
  const [resolution, setResolution] = useState<string | null>(null);
  const [latency, setLatency] = useState<number | null>(null);
  const [previousBytesReceived, setPreviousBytesReceived] = useState<number | null>(null);
  const [previousTimestamp, setPreviousTimestamp] = useState<number | null>(null);

  useEffect(() => {
    const video = videoRef.current;
    if (!video) return;
    video.srcObject = stream;
    video.autoplay = true;
    video.controls = false;
    video.width = 640;
    video.height = 480;

    const handleTrackAdded = () => {
      video.srcObject = stream;
    };

    stream.getTracks().forEach(track => {
      track.addEventListener('ended', handleTrackAdded);
    });

    return () => {
      stream.getTracks().forEach(track => {
        track.removeEventListener('ended', handleTrackAdded);
      });
      console.log(`Cleaning Up Video ${streamId}`);
    };
  }, [stream, streamId]);

  // Function to get connection stats
  const getConnectionStats = useCallback(async () => {
    if (!pc) return;
    try {
      const stats = await pc.getStats(null);
      stats.forEach(report => {
        console.log(report);
        if (report.type === 'inbound-rtp' && report.kind === 'video') {
          if (report.trackIdentifier !== stream.getVideoTracks()[0]?.id) {
            return;
          }
          // Bitrate calculation
          const bytesReceived = report.bytesReceived;
          const timestamp = report.timestamp;
          if (previousBytesReceived === null) {
            setPreviousBytesReceived(bytesReceived);
            setPreviousTimestamp(timestamp);
            return;
          }
          const bytesDiff = bytesReceived - previousBytesReceived;
          const timeDiff = timestamp - previousTimestamp!;
          const bitrate = (bytesDiff * 8) / (timeDiff); // bits per millisecond
          const bitrateKbps = bitrate; // kilobits per second
          const bitrateMbps = bitrateKbps / 1000; // megabits per second
          setBitrate(bitrateMbps);
          setPreviousBytesReceived(bytesReceived);
          setPreviousTimestamp(timestamp);

          // Latency calculation
   
          // FPS and resolution
          setFps(report.framesPerSecond);
          if (report.frameWidth && report.frameHeight) {
            setResolution(`${report.frameWidth}x${report.frameHeight}`);
          }
        }

     
      });
    } catch (e) {
      console.error("Error getting stats:", e);
    }
  }, [pc, previousBytesReceived, previousTimestamp, stream]);

  // Call getConnectionStats periodically
  useEffect(() => {
    const intervalId = setInterval(() => {
      getConnectionStats();
    }, 1000);
    return () => clearInterval(intervalId);
  }, [getConnectionStats]);

  return (
    <div id={streamId} className="relative">
      <video ref={videoRef} playsInline className="w-full h-full max-h-[70vh] object-contain" />
      <div className="absolute bottom-2 left-2 bg-gray-800/80 text-white p-2 rounded-lg text-sm flex gap-4">
        <span>{streamId}</span>
        {bitrate !== null && <span>{bitrate.toFixed(1)} Mbps</span>}
        {fps !== null && <span>{Math.round(fps)} FPS</span>}
        {resolution !== null && <span>{resolution}</span>}
        {latency !== null && <span>{latency.toFixed(1)} ms</span>}
      </div>
    </div>
  );
});

const StartStream = ({send_json,message}:{send_json: (msg: RTCSessionDescription | RTCIceCandidate | RTCSessionDescriptionInit) => void,message?:any}) => {
  const [remoteStreams, setRemoteStreams] = useState<Map<string, MediaStream>>(new Map());
  const [selectedVideo, setSelectedVideo] = useState<string | null>("both"); // Default to single
  const [pc, setPc] = useState<RTCPeerConnection | null>(null);
  const videoContainerRef = useRef<HTMLDivElement | null>(null);
  const [isFullscreen, setIsFullscreen] = useState(false);

  const [connectionState, setConnectionState] = useState<string | null>(null);
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
        console.log("Received remote stream:", stream);
        const streamId = stream.id;
        setRemoteStreams((prevStreams) => {
          const newStreams = new Map(prevStreams);
          newStreams.set(streamId, stream);
          return newStreams;
        });
      };
      // Log the ICE connection state
      peerConnection.oniceconnectionstatechange = () => {
        log(`ICE Connection State: ${peerConnection.iceConnectionState}`)
        setConnectionState(peerConnection.iceConnectionState);
      };
      peerConnection.onicecandidate = (event) => {
        if (event.candidate) {
          console.log('New ICE candidate:', event.candidate);
          send_json(event.candidate)
        }
      };
      // Handle ICE gathering state event
      peerConnection.onicegatheringstatechange = async () => {
        console.log("ICE gathering state changed to:", peerConnection.iceGatheringState);  // Debugging
      };
      // Set up transceivers and create an offer
      peerConnection.addTransceiver('video', { direction: 'recvonly' });
      peerConnection.addTransceiver('video', { direction: 'recvonly' });
      try {
        const offer = await peerConnection.createOffer();
        await peerConnection.setLocalDescription(offer);
        send_json(offer)
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

  useEffect(()=>{
    if (!pc || !message){
      return;
    }
    console.log(message)
    if (message.candidate != undefined){
      console.log("adding canidate")
      pc?.addIceCandidate(message)
    }else{
      console.log("Setting Remote Description")
      pc.setRemoteDescription(message)
    }
  },[message,pc])


  const streamIds = Array.from(remoteStreams.keys());
  const bothStreamsReady = streamIds.length >= 2 && connectionState == "connected" &&streamIds.every(id => remoteStreams.get(id) !== undefined);
  const singleStreamReady = selectedVideo !== "both" && connectionState == "connected"&& selectedVideo !== null && remoteStreams.get(selectedVideo) !== undefined;
  return (
    <Card className='col-span-3 row-span-2'>
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
          {singleStreamReady ? (
            <RemoteVideo key={selectedVideo!}  streamId={selectedVideo!} stream={remoteStreams.get(selectedVideo!)!} pc={pc} />
          ) : bothStreamsReady ? (
            <div className="flex w-full h-full">
              <div className="w-1/2 h-full">
                <RemoteVideo key={streamIds[0]} streamId={streamIds[0]} stream={remoteStreams.get(streamIds[0])!} pc={pc} />
              </div>
              <div className="w-1/2 h-full">
                <RemoteVideo key={streamIds[1]} streamId={streamIds[1]} stream={remoteStreams.get(streamIds[1])!} pc={pc} />
              </div>
            </div>
          ) : (
            <div className="text-white text-center py-4 h-[480px]">
              {streamIds.length === 0 ? "No streams available" : "Loading streams..."}
            </div>
          )}
        </div>
      </CardContent>
    </Card>
  );
};

export default StartStream;
