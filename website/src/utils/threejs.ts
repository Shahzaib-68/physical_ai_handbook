// Three.js integration for 3D visualization demos
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';
import { getAppConfig } from '../utils/config';
import { DemoExecutionError } from '../utils/validation';

// Interface for Three.js scene configuration
interface SceneConfig {
  containerId?: string;
  width?: number;
  height?: number;
  backgroundColor?: number;
  antialias?: boolean;
}

// Interface for Three.js execution result
interface ThreeJSExecutionResult {
  sceneId: string;
  executionTime: number;
  stats: {
    objects: number;
    geometries: number;
    textures: number;
  };
}

// Class to manage Three.js scenes
export class ThreeSceneManager {
  private scene: THREE.Scene | null = null;
  private camera: THREE.PerspectiveCamera | null = null;
  private renderer: THREE.WebGLRenderer | null = null;
  private controls: OrbitControls | null = null;
  private container: HTMLElement | null = null;
  private animationId: number | null = null;
  private objects: THREE.Object3D[] = [];
  
  // Initialize a Three.js scene
  async init(containerId: string, config: SceneConfig = {}): Promise<void> {
    try {
      // Get container element
      this.container = document.getElementById(containerId);
      if (!this.container) {
        throw new DemoExecutionError(`Container with id '${containerId}' not found`);
      }
      
      // Create scene
      this.scene = new THREE.Scene();
      this.scene.background = new THREE.Color(config.backgroundColor || 0xf0f8ff); // Light blue
      
      // Create camera
      const width = config.width || this.container.clientWidth;
      const height = config.height || this.container.clientHeight;
      this.camera = new THREE.PerspectiveCamera(75, width / height, 0.1, 1000);
      this.camera.position.z = 5;
      
      // Create renderer
      this.renderer = new THREE.WebGLRenderer({ antialias: config.antialias !== false });
      this.renderer.setSize(width, height);
      this.renderer.setClearColor(0xf0f8ff);
      
      // Add renderer to container
      this.container.appendChild(this.renderer.domElement);
      
      // Add orbit controls
      if (this.camera && this.container) {
        this.controls = new OrbitControls(this.camera, this.container);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.25;
      }
      
      // Add lighting
      this.addDefaultLighting();
      
      // Handle window resize
      window.addEventListener('resize', this.handleResize);
    } catch (error) {
      console.error('Error initializing Three.js scene:', error);
      throw new DemoExecutionError(`Failed to initialize Three.js scene: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  }
  
  // Add default lighting to the scene
  private addDefaultLighting(): void {
    if (!this.scene) return;
    
    // Ambient light
    const ambientLight = new THREE.AmbientLight(0x404040, 0.6);
    this.scene.add(ambientLight);
    
    // Directional light
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(1, 1, 1);
    this.scene.add(directionalLight);
    
    // Hemisphere light
    const hemisphereLight = new THREE.HemisphereLight(0xffffbb, 0x080820, 0.5);
    this.scene.add(hemisphereLight);
  }
  
  // Handle window resize
  private handleResize = () => {
    if (!this.camera || !this.renderer || !this.container) return;
    
    const width = this.container.clientWidth;
    const height = this.container.clientHeight;
    
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
  };
  
  // Add a geometry to the scene
  addGeometry(
    geometry: THREE.BufferGeometry,
    material: THREE.Material,
    position?: THREE.Vector3
  ): THREE.Mesh {
    if (!this.scene) {
      throw new DemoExecutionError('Scene not initialized');
    }
    
    const mesh = new THREE.Mesh(geometry, material);
    
    if (position) {
      mesh.position.copy(position);
    }
    
    this.scene.add(mesh);
    this.objects.push(mesh);
    
    return mesh;
  }
  
  // Add a primitive shape to the scene
  addPrimitive(
    type: 'box' | 'sphere' | 'plane' | 'torus' | 'cone' | 'cylinder',
    size: number,
    material?: THREE.Material,
    position?: THREE.Vector3
  ): THREE.Mesh {
    let geometry: THREE.BufferGeometry;
    
    switch (type) {
      case 'box':
        geometry = new THREE.BoxGeometry(size, size, size);
        break;
      case 'sphere':
        geometry = new THREE.SphereGeometry(size, 32, 32);
        break;
      case 'plane':
        geometry = new THREE.PlaneGeometry(size, size);
        break;
      case 'torus':
        geometry = new THREE.TorusGeometry(size, size * 0.3, 16, 100);
        break;
      case 'cone':
        geometry = new THREE.ConeGeometry(size, size * 2, 32);
        break;
      case 'cylinder':
        geometry = new THREE.CylinderGeometry(size, size, size * 2, 32);
        break;
      default:
        throw new DemoExecutionError(`Unknown primitive type: ${type}`);
    }
    
    const defaultMaterial = material || new THREE.MeshPhongMaterial({ 
      color: 0x00ff00,
      wireframe: false
    });
    
    return this.addGeometry(geometry, defaultMaterial, position);
  }
  
  // Start animation loop
  startAnimation(animateFn?: (deltaTime: number) => void): void {
    if (!this.renderer || !this.scene || !this.camera) {
      throw new DemoExecutionError('Scene not properly initialized');
    }
    
    let lastTime = 0;
    
    const animate = (time: number) => {
      const deltaTime = (time - lastTime) / 1000;
      lastTime = time;
      
      // Update controls
      if (this.controls) {
        this.controls.update();
      }
      
      // Custom animation callback
      if (animateFn) {
        animateFn(deltaTime);
      }
      
      // Render scene
      this.renderer?.render(this.scene, this.camera);
      
      // Continue animation loop
      this.animationId = requestAnimationFrame(animate);
    };
    
    // Start animation
    this.animationId = requestAnimationFrame(animate);
  }
  
  // Stop animation loop
  stopAnimation(): void {
    if (this.animationId) {
      cancelAnimationFrame(this.animationId);
      this.animationId = null;
    }
  }
  
  // Get scene statistics
  getStats(): { objects: number; geometries: number; textures: number } {
    if (!this.scene) {
      return { objects: 0, geometries: 0, textures: 0 };
    }
    
    // Count objects in scene
    const objects = this.scene.children.length;
    
    // This is a simplified counting - in a real implementation you'd have more detailed stats
    return {
      objects,
      geometries: objects, // Approximate
      textures: 0 // Would need to count materials with textures
    };
  }
  
  // Execute a 3D visualization demo
  async executeDemo(
    containerId: string,
    config: SceneConfig = {},
    setupFn?: (scene: ThreeSceneManager) => void
  ): Promise<ThreeJSExecutionResult> {
    const startTime = Date.now();
    
    try {
      // Initialize scene
      await this.init(containerId, config);
      
      // Run custom setup if provided
      if (setupFn) {
        setupFn(this);
      }
      
      // Start animation
      this.startAnimation();
      
      const endTime = Date.now();
      
      return {
        sceneId: containerId,
        executionTime: endTime - startTime,
        stats: this.getStats()
      };
    } catch (error) {
      console.error('Error executing Three.js demo:', error);
      throw new DemoExecutionError(`Three.js execution error: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  }
  
  // Dispose of the scene and clean up resources
  dispose(): void {
    // Stop animation
    this.stopAnimation();
    
    // Dispose renderer
    this.renderer?.dispose();
    
    // Dispose geometries and materials
    this.objects.forEach(obj => {
      if (obj instanceof THREE.Mesh) {
        obj.geometry.dispose();
        if (Array.isArray(obj.material)) {
          obj.material.forEach(mat => (mat as THREE.Material).dispose());
        } else {
          (obj.material as THREE.Material).dispose();
        }
      }
    });
    
    // Remove event listeners
    window.removeEventListener('resize', this.handleResize);
    
    // Clear references
    this.scene = null;
    this.camera = null;
    this.renderer = null;
    this.controls = null;
    this.container = null;
    this.objects = [];
  }
}

// Helper function to create common materials
export const createMaterial = (
  type: 'basic' | 'lambert' | 'phong' | 'standard' | 'physical',
  options: any = {}
): THREE.Material => {
  switch (type) {
    case 'basic':
      return new THREE.MeshBasicMaterial({
        color: options.color || 0x00ff00,
        wireframe: options.wireframe || false
      });
    case 'lambert':
      return new THREE.MeshLambertMaterial({
        color: options.color || 0x00ff00,
        wireframe: options.wireframe || false
      });
    case 'phong':
      return new THREE.MeshPhongMaterial({
        color: options.color || 0x00ff00,
        wireframe: options.wireframe || false,
        shininess: options.shininess || 30
      });
    case 'standard':
      return new THREE.MeshStandardMaterial({
        color: options.color || 0x00ff00,
        roughness: options.roughness || 0.5,
        metalness: options.metalness || 0.5
      });
    case 'physical':
      return new THREE.MeshPhysicalMaterial({
        color: options.color || 0x00ff00,
        roughness: options.roughness || 0.5,
        metalness: options.metalness || 0.5
      });
    default:
      return new THREE.MeshPhongMaterial({ color: 0x00ff00 });
  }
};

// Validate Three.js code (basic validation)
export const validateThreeJS = (code: string): string[] => {
  const errors: string[] = [];

  // Check for potentially problematic Three.js operations
  const suspiciousPatterns = [
    /new\s+THREE\.WebGLRenderer\(\)\.setSize\([^,]+,[^,]+\)/,  // Large canvas sizes
    /Array\(\d{6,}\)\.fill\(\)/,  // Very large arrays
  ];

  for (const pattern of suspiciousPatterns) {
    if (pattern.test(code)) {
      errors.push(`Code contains pattern that may cause performance issues: ${pattern}`);
    }
  }

  return errors;
};