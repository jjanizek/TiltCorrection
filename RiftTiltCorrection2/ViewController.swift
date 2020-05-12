//
//  ViewController.swift
//  RiftTiltCorrection2
//
//  Created by Joseph Janizek on 5/8/20.
//  Copyright © 2020 Joseph Janizek. All rights reserved.
//

import UIKit
import CoreMotion
import Accelerate
import simd

class ViewController: UIViewController {
    
    @IBOutlet weak var mainLabel: UILabel!
    let motionManager = CMMotionManager()
    var timer: Timer!
    
    let biasX = -0.0015233016100131723
    let biasY = 0.0125554432954707
    let biasZ = -0.0007909691420200371
    
    let gyroBiasX = 0.004415220879810092
    let gyroBiasY = -0.008042619335614957
    let gyroBiasZ = 0.010417670011691955

    let origin: simd_double3 = simd_double3(x: 0.0, y: 0.0, z: 1.0)
    
    var qHat: simd_quatd = simd_quatd(angle: 1.0,
                                      axis: simd_double3(x: 0.0,
                                                         y: 0.0,
                                                         z: 0.0))
    
    let deltaT = 1.0 / 60.0

    override func viewDidLoad() {
        super.viewDidLoad()
        // Do any additional setup after loading the view.
        motionManager.startAccelerometerUpdates()
        motionManager.startGyroUpdates()
        
        timer = Timer.scheduledTimer(timeInterval: self.deltaT, target: self, selector: #selector(ViewController.update), userInfo: nil, repeats: true)
    }
    
    @objc func update() {
    if let gyroData = motionManager.gyroData {
        let gyroX = gyroData.rotationRate.x - self.gyroBiasX
        let gyroY = gyroData.rotationRate.y - self.gyroBiasY
        let gyroZ = gyroData.rotationRate.z - self.gyroBiasZ
        
        let gyroOmega: simd_double3 = simd_double3(x: gyroX,
                                                   y: gyroY,
                                                   z: gyroZ)
        
        let ell: Double = simd_length(gyroOmega)
        let theta = ell * self.deltaT
        
        let gyroQuaternion: simd_quatd = simd_quatd(angle: theta,
                                                    axis: simd_normalize(gyroOmega))
        
        self.qHat = simd_mul(self.qHat, gyroQuaternion)
    }
    
    if let accelerometerData = motionManager.accelerometerData {
        let accelX = accelerometerData.acceleration.x - self.biasX
        let accelY = accelerometerData.acceleration.y - self.biasY
        let accelZ = accelerometerData.acceleration.z - self.biasZ
        
        let aHat: simd_double3 = simd_double3(x: accelX,
                                              y: accelY,
                                              z: accelZ)
        
        let aQuat: simd_quatd = simd_quatd(angle: Double.pi,
                                           axis:simd_normalize(aHat))
        
        let globalA: simd_quatd = simd_mul(simd_mul(self.qHat, aQuat),simd_inverse(self.qHat))
        
        let tiltError: Double = acos(abs(simd_normalize(globalA.axis).z))
        let tiltAxis: simd_double3 = simd_double3(x: globalA.axis.y, y: -globalA.axis.x, z: 0.0)
        let complementaryFilter: simd_quatd = simd_quatd(angle: -0.02 * tiltError,
                                                         axis: tiltAxis)
        self.qHat = simd_mul(complementaryFilter, self.qHat)
//        let justAccelerometer: Double = acos(simd_dot(-self.origin, simd_normalize(aHat)))
        }
        
    let rotatedOrigin: simd_double3 = simd_normalize(self.qHat.act(origin))
    let degreeAngle = (acos(simd_dot(origin, rotatedOrigin))*180)/Double.pi
    self.mainLabel.text = String(degreeAngle)
    }


}

