"""Bind native passive evidence to the live arm mechanics used downstream."""
import math


ARRAYS = {k: 2 for k in ('virtual_mass', 'virtual_damping', 'virtual_stiffness',
    'displacement_limit_m', 'velocity_limit_m_s', 'acceleration_limit_m_s2',
    'relief_inertia', 'relief_damping', 'relief_stiffness', 'relief_displacement_rad',
    'relief_velocity_rad_s', 'relief_acceleration_rad_s2')}
ARRAYS['arm_task_metric'] = 6
SCALARS = ('virtual_yaw_inertia', 'virtual_yaw_damping', 'virtual_yaw_stiffness',
    'yaw_displacement_limit_rad', 'yaw_velocity_limit_rad_s', 'yaw_acceleration_limit_rad_s2',
    'pose_gain_s_inv', 'joint_request_speed_rad_s', 'joint_acceleration_rad_s2', 'joint_margin_rad',
    'height_error_limit_m', 'rotation_error_limit_rad', 'wrench_force_limit_N', 'wrench_torque_limit_Nm')
BOOLS = ('rotational_relief_enabled', 'arm_yaw_admittance_enabled', 'native_grasp_monitor_required', 'allow_ungrasped_test')
KEYS = (*ARRAYS, *SCALARS, *BOOLS)


def validate(policy):
    if not isinstance(policy, dict) or set(policy) != set(KEYS):
        raise ValueError('complete measured native arm mechanics required')
    for name, size in ARRAYS.items():
        value = policy[name]
        if not isinstance(value, list) or len(value) != size or not all(type(v) in (int,float) and math.isfinite(v) and v>0. for v in value):
            raise ValueError('invalid native arm mechanics: '+name)
    for name in SCALARS:
        value = policy[name]
        if type(value) not in (int,float) or not math.isfinite(value) or value<=0.:
            raise ValueError('invalid native arm mechanics: '+name)
    if any(type(policy[k]) is not bool for k in BOOLS):
        raise ValueError('actual native arm policy boolean required')
    if policy['allow_ungrasped_test'] or not policy['native_grasp_monitor_required'] or not policy['arm_yaw_admittance_enabled']:
        raise ValueError('native arm requires grasp monitoring and own yaw compliance')
    return policy


def read_live(node, wait):
    from rcl_interfaces.srv import GetParameters
    from rcl_interfaces.msg import ParameterType as T
    client = node.create_client(GetParameters, '/follower/follower_yaw_whole_body/get_parameters')
    try:
        wait(client.service_is_ready, 10.)
        future = client.call_async(GetParameters.Request(names=list(KEYS)))
        wait(future.done, 10.)
        values = future.result().values
        if len(values) != len(KEYS):
            raise ValueError('native arm RPC response incomplete')
        policy = {}
        for name, value in zip(KEYS, values):
            expected = T.PARAMETER_DOUBLE_ARRAY if name in ARRAYS else T.PARAMETER_BOOL if name in BOOLS else T.PARAMETER_DOUBLE
            if value.type != expected:
                raise ValueError('native arm parameter RPC type mismatch: '+name)
            policy[name] = list(value.double_array_value) if name in ARRAYS else value.bool_value if name in BOOLS else value.double_value
        return validate(policy)
    finally:
        node.destroy_client(client)


def transfer(result, certificate, params):
    """No guessed defaults: the preceding live RPC is the authority."""
    actual = validate(result.get('native_arm_policy'))
    if certificate.get('native_arm_policy') != actual:
        raise ValueError('native certificate and measured arm mechanics differ')
    params.update(actual)
    return actual
