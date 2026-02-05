"use client"

import { Nav } from "../components/nav";
import { Notifications } from "../components/notifications";
import { View } from 'react-native';

// video stream
import { FoxgloveViewer } from "@foxglove/embed-react";


export function Home() {
return (
  <>

  <Nav />

  <hr/>

  <div className = "header">
    <h1> La Princesa Rover </h1>
      <h2> Solis Rover Project | 2025 - 26 </h2>
  <p>homepage.tsx</p>
  </div>

<div className = "stream-align">
  <div className="video-stream-div"></div>
  <div className="video-stream-div"></div>
  <div className="video-stream-div"></div>
</div>

  </>
)
}