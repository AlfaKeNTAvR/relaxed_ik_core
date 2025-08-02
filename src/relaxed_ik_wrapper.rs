use crate::relaxed_ik;
use crate::utils_rust::subscriber_utils::EEPoseGoalsSubscriber;
use nalgebra::{Vector3, UnitQuaternion, Quaternion, Translation3, Isometry3};
use std::os::raw::{*};
use std::sync::{Arc, Mutex};

/// Global mutable static storage for RelaxedIK instance
static mut R: Option<Mutex<relaxed_ik::RelaxedIK>> = None;

#[no_mangle]
pub unsafe extern "C" fn initialize_relaxed_ik(settings_file_ptr: *const c_char, mode: c_int) {
    assert!(!settings_file_ptr.is_null(), "Settings file path is null!");

    let c_str = std::ffi::CStr::from_ptr(settings_file_ptr);
    let settings_file = c_str.to_str().expect("Invalid UTF-8 string").to_string();

    println!("[RelaxedIK] Initializing with settings file: {}", settings_file);

    let ik = relaxed_ik::RelaxedIK::from_settings_file(settings_file, mode as usize);
    R = Some(Mutex::new(ik));
}

#[no_mangle]
pub unsafe extern "C" fn dynamic_obstacle_cb(name: *const c_char, pos_arr: *const c_double, quat_arr: *const c_double) {
    assert!(!name.is_null(), "Empty name!");
    assert!(!pos_arr.is_null(), "Null pointer for pos!");
    assert!(!quat_arr.is_null(), "Null pointer for quat!");

    let c_str = std::ffi::CStr::from_ptr(name);
    let name_str = c_str.to_str().expect("Not a valid UTF-8 string");

    let pos_slice: &[c_double] = std::slice::from_raw_parts(pos_arr, 3);
    let quat_slice: &[c_double] = std::slice::from_raw_parts(quat_arr, 4);

    let pos_vec = pos_slice.to_vec();
    let quat_vec = quat_slice.to_vec();

    let ts = Translation3::new(pos_vec[0], pos_vec[1], pos_vec[2]);
    let tmp_q = Quaternion::new(quat_vec[3], quat_vec[0], quat_vec[1], quat_vec[2]);
    let rot = UnitQuaternion::from_quaternion(tmp_q);
    let pos = Isometry3::from_parts(ts, rot);

    unsafe {
        R.as_ref()
            .expect("RelaxedIK not initialized!")
            .lock()
            .unwrap()
            .vars
            .env_collision
            .update_dynamic_obstacle(name_str, pos);
    }
}

#[no_mangle]
pub unsafe extern "C" fn solve(pos_arr: *const c_double, pos_length: c_int,
                                quat_arr: *const c_double, quat_length: c_int) -> relaxed_ik::Opt {
    assert!(!pos_arr.is_null(), "Null pointer for pos goals!");
    assert!(!quat_arr.is_null(), "Null pointer for quat goals!");

    let pos_slice: &[c_double] = std::slice::from_raw_parts(pos_arr, pos_length as usize);
    let quat_slice: &[c_double] = std::slice::from_raw_parts(quat_arr, quat_length as usize);

    let pos_vec = pos_slice.to_vec();
    let quat_vec = quat_slice.to_vec();

    let ja = solve_helper(pos_vec, quat_vec);
    let ptr = ja.as_ptr();
    let len = ja.len();
    std::mem::forget(ja); // Prevents deallocation; memory is now owned externally

    relaxed_ik::Opt { data: ptr, length: len as c_int }
}

fn solve_helper(pos_goals: Vec<f64>, quat_goals: Vec<f64>) -> Vec<f64> {
    let arc = Arc::new(Mutex::new(EEPoseGoalsSubscriber::new()));
    let mut g = arc.lock().unwrap();

    let num_chains = unsafe {
        R.as_ref()
            .expect("RelaxedIK not initialized!")
            .lock()
            .unwrap()
            .vars
            .robot
            .num_chains
    };

    for i in 0..num_chains {
        g.pos_goals.push(Vector3::new(pos_goals[3 * i], pos_goals[3 * i + 1], pos_goals[3 * i + 2]));
        let tmp_q = Quaternion::new(quat_goals[4 * i + 3], quat_goals[4 * i], quat_goals[4 * i + 1], quat_goals[4 * i + 2]);
        g.quat_goals.push(UnitQuaternion::from_quaternion(tmp_q));
    }

    let x = unsafe {
        R.as_ref()
            .expect("RelaxedIK not initialized!")
            .lock()
            .unwrap()
            .solve(&g)
    };

    x
}

#[no_mangle]
pub unsafe extern "C" fn reset(joint_state: *const c_double, joint_state_length: c_int) {
    let x_slice: &[c_double] = std::slice::from_raw_parts(joint_state, joint_state_length as usize);
    let x_vec = x_slice.to_vec();

    unsafe {
        R.as_ref()
            .expect("RelaxedIK not initialized!")
            .lock()
            .unwrap()
            .reset(x_vec);
    }
}
