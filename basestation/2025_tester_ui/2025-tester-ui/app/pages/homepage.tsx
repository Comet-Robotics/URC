"use client"

import { Nav } from "../components/nav";
import { Footer } from "../components/footer"

// video stream
import { FoxgloveViewer } from "@foxglove/embed-react";


export function Home() {

  // Returns the actual chompage component

  return (
    <>
      <Nav />

      <hr/>

      <div className = "header">
        <h1> La Princesa Rover </h1>
          <h2> Solis Rover Project | 2025 - 26 </h2>
      <p>homepage.tsx</p>
      </div>

      <h3 className="video-title">Video Streams</h3>

      <div className = "stream-align">
        <div className="video-stream-div"></div>
        <div className="video-stream-div"></div>
        <div className="video-stream-div"></div>
      </div>

      <Footer />
    </>
  )
}