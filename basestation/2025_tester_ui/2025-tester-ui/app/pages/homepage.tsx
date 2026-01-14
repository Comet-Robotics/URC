"use client"

import { Nav } from "../components/nav";
import { Card } from "../components/card";

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

  <div>
      <div className = "block">
        <button> battery life </button>
      </div>

      <div className = "block">
        <h3> connection status </h3>
      </div>

      <div className = "block">
        <h3>controller data</h3>
      </div>
  </div>
  
  </>
)
}