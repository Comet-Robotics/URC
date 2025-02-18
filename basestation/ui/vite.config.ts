import path from "path";
import react from "@vitejs/plugin-react";
import { defineConfig } from "vite";

export default defineConfig({
  plugins: [react()],
  build: {
    minify: false, // Disable minification
    sourcemap: true, // Generate sourcemaps for easier debugging
    // target: 'esnext', // Or 'es2020', 'es2019', etc.  This controls the output JavaScript version.
  },
  resolve: {
    alias: {
      "@": path.resolve(__dirname, "./src"),
    },
  },
});
