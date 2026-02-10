"use client"

import { Nav } from "../components/nav";
import { Footer } from "../components/footer"

export default function Page() {
  return (
  <>
  <Nav />
  <hr/>
  <div className = "header">
    <h1> La Princesa Rover </h1>
      <h2> Science Payload </h2>
  <p>science.tsx</p>

<h3 className="video-title">Hyperspectral Camera</h3>
  <div className = "stream-align">
  <div className="hyperspec"></div>
</div>
  </div>
  
    <Footer />
  </>
)
}