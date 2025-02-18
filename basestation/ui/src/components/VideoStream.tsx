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

interface RemoteVideoProps {
  streamId: string;
  stream: MediaStream;
  pc: RTCPeerConnection | null; // Pass the peer connection
}

// eslint-disable-next-line react/display-name
const RemoteVideo = React.memo(({ streamId, stream, pc }: RemoteVideoProps) => {
  const videoRef = useRef<HTMLVideoElement>(null);
  const [bitrate, setBitrate] = useState<number | null>(null);
  const [fps, setFps] = useState<number | null>(null);
  const [resolution, setResolution] = useState<string | null>(null);
  const [previousBytesReceived, setPreviousBytesReceived] = useState<number | null>(null);
  const [previousTimestamp, setPreviousTimestamp] = useState<number | null>(null);
  const [noDataReceived, setNoDataReceived] = useState(true);
  const [lastDataTime, setLastDataTime] = useState<number>(Date.now());

  useEffect(() => {
    if (!videoRef.current) return;
    videoRef.current.srcObject = stream;
    videoRef.current.autoplay = true;
    videoRef.current.controls = false;
    videoRef.current.width = 640;
    videoRef.current.height = 480;
    videoRef.current.muted = true;

    return () => {
     
      console.log(`Cleaning Up Video ${streamId}`);
    };
  }, [stream, streamId]);

  // Function to get connection stats
  const getConnectionStats = useCallback(async () => {
    if (!pc) return;
    try {
      const stats = await pc.getStats(null);
      let hasReceivedData = false;
      
      stats.forEach(report => {
        if (report.type === 'inbound-rtp' && report.kind === 'video') {
          if (report.trackIdentifier !== stream.getVideoTracks()[0]?.id) {
            return;
          }
          
 

          // Bitrate calculation
          const bytesReceived = report.bytesReceived;
          if (bytesReceived > 0 && bytesReceived !== previousBytesReceived) {
            hasReceivedData = true;
            setLastDataTime(Date.now());
          }
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

      // If it's been more than 5 seconds since last data
      if (Date.now() - lastDataTime > 5000) {
        setNoDataReceived(true);
      } else if (hasReceivedData) {
        setNoDataReceived(false);
      }

    } catch (e) {
      console.error("Error getting stats:", e);
    }
  }, [pc, previousBytesReceived, previousTimestamp, stream, lastDataTime]);

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
      {noDataReceived && (
        <div className="absolute inset-0 flex items-center justify-center bg-black/70 text-white">
          <span className="text-lg">No video data received...</span>
        </div>
      )}
      <div className="absolute bottom-2 left-2 bg-gray-800/80 text-white p-2 rounded-lg text-sm flex gap-4">
        <span>{streamId}</span>
        {bitrate !== null && <span>{bitrate.toFixed(1)} Mbps</span>}
        {fps !== null && <span>{Math.round(fps)} FPS</span>}
        {resolution !== null && <span>{resolution}</span>}
      </div>
    </div>
  );
});

// eslint-disable-next-line @typescript-eslint/no-explicit-any
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
      await navigator.mediaDevices.getUserMedia({ audio: true})
      
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
            <RemoteVideo  streamId={selectedVideo!} stream={remoteStreams.get(selectedVideo!)!} pc={pc} />
          ) : bothStreamsReady ? (
            <div className="flex w-full h-full">
              <div className="w-1/2 h-full">
                <RemoteVideo  streamId={streamIds[0]} stream={remoteStreams.get(streamIds[0])!} pc={pc} />
              </div>
              <div className="w-1/2 h-full">
                <RemoteVideo  streamId={streamIds[1]} stream={remoteStreams.get(streamIds[1])!} pc={pc} />
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
