//global variables
var global_i = 0;
const global_max_i = 50;
const PI = Math.PI;

//Key Variables
var L_Arrow_Status = 0;
var U_Arrow_Status = 0;
var R_Arrow_Status = 0;
var D_Arrow_Status = 0;
var num0_Pad_Status = 0;

// Machine parametres
const L = [3, 3, 1.45];
const PID1 = [0.005, 0.5, 0.01];
const PID2 = [0.2, 0.5, 0.1];

//Control variables
const manual_step = 1;
var P_obj = [6, 0];
var theta1 = [0, 0, 0, 0]; //Current, Objective, Last_Error, Cumulative_Error
var theta2 = [0, 0, 0, 0];

function process_angle(raw_angle, offset) {

    return -PI * (raw_angle - offset) * 2;

} 

function throtled_var_print(num_to_print, num_name) {

    if (global_i == global_max_i) {

        print(num_name + ": " + str(num_to_print));
        //global_i = 0;

    }

}

function throtled_str_print(str_to_print) {

    if (global_i == global_max_i) {

        print(str_to_print);
        //global_i = 0;

    }

}

function PID(current_var, objective_var, last_error, PID_params) {

    Kp = PID_params[0];
    Ki = PID_params[1];
    Kd = PID_params[2];

    //throtled_var_print(current_var, "Current");
    //throtled_var_print(objective_var, "Objective");

    // var error = objective_var - current_var;
    var i_error = last_error + error;

    if (Math.abs(error) < 0.05) {

        error = 0;
        i_error = 0;

    }

    // throtled_var_print(error, "Error: ");
    var u = Kp*(error) + Kd*(error-last_error)*60 + Ki*i_error;
    // throtled_var_print(u, "U");
    // throtled_var_print(Kp*(error), "Kp");
    // throtled_var_print(Kd*(error-last_error)*60, "Kd");
    // throtled_var_print(Ki*i_error, "Ki");

    return [u, error, i_error];

}

function move_r_joint(matrix_angle, PIO_index, PID_used) {

    var output = 0;

    error = matrix_angle[1] - matrix_angle[0];
    if (Math.abs(error > PI)) {

        error = -(2*PI - matrix_angle[1] + matrix_angle[0]);

    }
    //throtled_var_print(error, "Error");
    //throtled_var_print(matrix_angle[0], "Current");
    //throtled_var_print(matrix_angle[1], "Objective");

    [output, matrix_angle[2], matrix_angle[3]] = PID(0, error, matrix_angle[2], PID_used);
    //throtled_var_print(matrix_angle[2], "Last Error");
    //throtled_var_print(output, "Output");

    if (output > 0) {

        //throtled_str_print("Moving Forwards");
        out(PIO_index+1, output);
        out(PIO_index+2, 0);

    }
    else {

        //throtled_str_print("Moving Backwards");
        out(PIO_index+1, 0);
        out(PIO_index+2, -output);

    }

}

function step_key_converter(id_key_listened, key_status, function_pressed) {

    if (key_status ==  0) { //not being pressed already
        
        if (in(id_key_listened) == 1) { //is it pressed now
            print("Key was pressed");
                key_status = 1;
                function_pressed();

        }

    }
    else{
        
        if (in(id_key_listened) == 0) {
            //print("Key was released");
            key_status = 0;

        }

    }

    return key_status;
}

function lineal_controller(desired_vector) {

    X = desired_vector[0];
    Y = desired_vector[1];
    //Z = desired_vector[2];

    throtled_var_print(X, "X");
    throtled_var_print(Y, "Y");

    X_2 = Math.pow(X,2);
    Y_2 = Math.pow(Y,2)

    R = Math.sqrt(X_2 + Y_2);

    if (R > L[0] + L[1]) {

        throtled_str_print("Out of Reach");

    }
    else {

        L0_2 = Math.pow(L[0],2);
        L1_2 = Math.pow(L[1],2); 

        beta = Math.atan2(Y,X);
        alpha = Math.acos((L1_2-L0_2-X_2-Y_2)/(-2*L[0]*R));
        //throtled_var_print((L1_2-L0_2-X_2-Y_2)/(-2*L[0]*R), "Pre_Alpha");
        
        //throtled_var_print(alpha, "Beta");
        //throtled_var_print(beta, "Alpha");

        theta1[1] = beta-alpha;
        theta2[1] = Math.acos((X_2+Y_2-L0_2-L1_2)/(2*L[0]*L[1]));
        

        //theta1[1] = 0;
        //theta2[1] = 0;

    }

}

// MAIN LOOP
while (true) {

    L_Arrow_Status = step_key_converter(6, L_Arrow_Status, () => { P_obj[1] += manual_step; });
    R_Arrow_Status = step_key_converter(7, R_Arrow_Status, () => { P_obj[1] -= manual_step; });
    U_Arrow_Status = step_key_converter(8, U_Arrow_Status, () => { P_obj[0] += manual_step; });
    D_Arrow_Status = step_key_converter(9, D_Arrow_Status, () => { P_obj[0] -= manual_step; });
    num0_Pad_Status = step_key_converter(10, num0_Pad_Status, () => { P_obj = [6, 0]; });

    theta1[0] = process_angle(in(0), 0);
    move_r_joint(theta1, 0, PID1);
    //throtled_var_print(theta1[2],"Last Error Theta1");

    theta2[0] = process_angle(in(3), in(0));
    move_r_joint(theta2, 3, PID2);

    if(R_Arrow_Status + L_Arrow_Status + U_Arrow_Status + D_Arrow_Status == 0) {

        //throtled_str_print("Updating Objective");
        lineal_controller(P_obj);

    }

    //Update Global Vars
    if (global_i == global_max_i) {

        global_i = 0;

    }
    global_i = global_i + 1;

    asleep()
}