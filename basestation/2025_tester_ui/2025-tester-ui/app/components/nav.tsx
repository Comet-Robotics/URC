// The Navbar Component

import { NavLink } from "react-router-dom";
import { IconButton } from '@mui/material';
import NotificationsIcon from '@mui/icons-material/Notifications';
// icons documentation: https://mui.com/material-ui/icons/

// The function that renders the Navbar

export function Nav() {
  return (
    <nav>

        {/* Navigation links */}

        <NavLink to="/" className={"navItems"} >Home</NavLink>
        <NavLink to="/science-payload" className={"navItems"}>Science Payload</NavLink>
        <NavLink to="/gyroscope" className={"navItems"}>Gyroscope</NavLink>
        <div className="notifs"><IconButton aria-label="notifs"><NotificationsIcon /></IconButton></div>
        {/* used to have  className="notifs" in iconbutton */}
    </nav>
  );
}   