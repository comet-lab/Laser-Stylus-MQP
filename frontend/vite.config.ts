import { defineConfig } from 'vite'

export default defineConfig({
  plugins: [],
  
  // Add this 'server' block
  server: {
    host: '0.0.0.0', // Allows access from your host machine
    port: 3000,
    watch: {
      usePolling: true // Enables polling for file changes
    }
  }
})