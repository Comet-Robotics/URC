"use client"

import { Nav } from "../components/nav";
import { Footer } from "../components/footer"
import HyperspectralVideoStream from "../components/hyperspectralVideoStream";

// Video streams configuration - update these URLs to match your camera backend
const CAMERA_STREAMS = {
  // camera1: process.env.NEXT_PUBLIC_CAMERA_1_URL || 'ws://localhost:8000/stream2''https://cdn.flowplayer.com/a30bd6bc-f98b-47bc-abf5-97633d4faea0/hls/de3f6ca7-2db3-4689-8160-0f574a5996ad/playlist.m3u8',
  camera2: process.env.NEXT_PUBLIC_CAMERA_2_URL || 'ws://localhost:8000/stream2',
};

export function Home() {

  // Returns the actual homepage component

  return (
    <>
      <Nav />

      <hr/>

      <div className = "header">
        <h1 className = "homepage-header"> La Princesa Rover </h1>
          <h2 className = "homepage-subheader"> Solis Rover Project</h2>
      </div>

      <h3 className="video-title">Video Streams</h3>

      <div className = "stream-align">
        <div className="video-stream-div">
          <HyperspectralVideoStream 
            streamId="Camera 1 - Stereoscope Video Stream" 
            streamUrl={CAMERA_STREAMS.camera1}
            width={640}
            height={480}
          />
        </div>
        <div className="video-stream-div">
          <HyperspectralVideoStream 
            streamId="Camera 2 - Lidar Camera" 
            streamUrl={CAMERA_STREAMS.camera2}
            width={640}
            height={480}
          />
        </div>
      </div>

      <Footer />
    </>
  )
}