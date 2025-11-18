export type SiteConfig = typeof siteConfig;

export const siteConfig = {
  name: "La Princesa Rover | SRP",
  description:
    "UTD URC Control Software",
  
    mainNav: [
    {
      title: "Homepage",
      // includes gyroscope, lidar sensor (essential), MAIN/steroscopic camera (consant feed, AI vision)
      // battery monitoring system (remaining battery percentage & battery time remaining), gps (distance from base station & satillite position), proximity sensor (distance from objects)
      // time > remaining and elapsed
      // current mission > status
      href: "/",
    },
    {
      title: "Science Payload",
      // temp (peak & average), moisture (peak & average), science payload > chemial distribution, whats in soil
      // ramen & vis spectometer > graph, peak, vibrational mode
      //hyperspectral camera
      href: "/",
    },
    {
      title: "Gyroscope",// BNOSX (model, plane value) & lidar (map of surroundings), IMU (direction)
      href: "/",
    }
  ],
  links: {
   
  },
};
