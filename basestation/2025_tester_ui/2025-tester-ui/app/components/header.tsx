// Header component

import { Nav } from "./nav";
import { Notifications } from "./notifications";

// This function changes the display on the notification sidebar

function notificationClick(){

  // AUGHHHHHHHHHHH

}

// The function that renders the Header

export function Header() {
  return (
    <div>
      <h1>Solis Rover Project </h1>
      <Nav />
      {/* Other stuff for the header here */}

      {/* Opens Notification Sidebar */}
      <button onClick={notificationClick}>Open Notification</button>
    </div>
  );
}