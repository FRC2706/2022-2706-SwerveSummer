// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.ErrorCode;

public class CheckError {
    private CheckError() {
    }

    public static void ctre(ErrorCode errorCode, String message) {
        if (errorCode != ErrorCode.OK) {
            throw new RuntimeException(String.format("%s: %s", message, errorCode.toString()));
        }
    }
}
