"use client"

import { Nav } from "../components/nav";
import { Notifications } from "../components/notifications";
import { View } from 'react-native';

let state = {
  isVisible: false
};

function toggleVisibility(){
  !state.isVisible;
};

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

  <div className = "big-block">
      <div className = "block">
        <h3> battery life </h3>
      </div>

      <div className = "block">
        <h3> connection status </h3>
      </div>

      <div className = "block">
        <h3>controller data</h3>
      </div>

  </div>

  <div className = "big-block">
      <div className = "block">
        <h3> battery life </h3>
        <hr/>
        <p>current percent: </p>
        <p>time remaining: </p>
      </div>

      <div className = "block">
        <h3> connection status </h3>
        <hr/>
        <p> strength: </p>
      </div>

      <div className = "block">
        <h3>controller data</h3>
        <hr/>
         <p>current battery: </p>
        <p>connection strength: </p>
      </div>
  </div>
  
  </>
)
}