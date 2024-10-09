from .diffusion_policy import get_resnet, replace_bn_with_gn, PushTImageDataset, ConditionalUnet1D,DDPMScheduler,PushTImageEnv,collections,normalize_data,unnormalize_data
import torch
import torch.nn as nn
import numpy as np
from tqdm.auto import tqdm
from skvideo.io import vwrite
import cv2
import time
from .findTclass import TShapeDetector
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray


from geometry_msgs.msg import Pose, PoseArray

class ActionPublisher(Node):
    def __init__(self):
        super().__init__('pusht_2')
        self.publisher_ = self.create_publisher(PoseArray, 'action_list', 10)

    def publish_actions(self, action_list):
        pose_array = PoseArray()
        poses = []
        for action in action_list:
            pose = Pose()
            # Assuming action is of the form [x, y]
            pose.position.x = round(float(action[0]) / 2000, 3)
            pose.position.y = round(float(action[1]) / 2000, 3)
            # pose.position.z = 0.093  # or some default value
            pose.orientation.w = 1.0  # or some default value
            poses.append(pose)
        pose_array.poses = poses
        self.publisher_.publish(pose_array)
        self.get_logger().info(f'Published {len(poses)} actions.')


def main(args=None):
    rclpy.init(args=args)
    action_publisher = ActionPublisher()

    vision_encoder =  get_resnet('resnet18')

    vision_encoder =  replace_bn_with_gn(vision_encoder)


    dataset_path = "/home/uk/ros2_ws/src/pusht/pusht/pusht_cchi_v7_replay.zarr.zip"
    pred_horizon=16
    obs_horizon=2
    action_horizon=8

    dataset =  PushTImageDataset(
        dataset_path=dataset_path,
        pred_horizon=pred_horizon,       # 16
        obs_horizon=obs_horizon,         #  2
        action_horizon=action_horizon    #  8
    )
    stats = dataset.stats

    vision_feature_dim =512
    lowdim_obs_dim=2
    obs_dim = vision_feature_dim + lowdim_obs_dim
    action_dim =2

    noise_pred_net= ConditionalUnet1D(
        input_dim=action_dim,
        global_cond_dim=obs_dim*obs_horizon
    )

    nets = nn.ModuleDict({
        'vision_encoder': vision_encoder,
        'noise_pred_net': noise_pred_net
    })

    num_diffusion_iters=100
    noise_scheduler= DDPMScheduler(
        num_train_timesteps=num_diffusion_iters,
        beta_schedule='squaredcos_cap_v2',
        clip_sample=True,
        prediction_type='epsilon'
    )

    device =torch.device('cpu')
    _=nets.to(device)

    ckpt_path = "/home/uk/ros2_ws/src/pusht/pusht/pusht_vision_100ep.ckpt"

    state_dict = torch.load(ckpt_path, map_location='cpu')
    ema_nets = nets
    ema_nets.load_state_dict(state_dict)
    print('Pretrained weights loaded.')


    max_steps=200
    env= PushTImageEnv()
    t_detector=TShapeDetector()

    while(True):
        # global cam_img
        angle,center=t_detector.detect_block_shape()
        if angle==None or center==None:
            print("anyting detect")
            time.sleep(2.0)
            continue
        # cam_img=t_detector.frame
        break


    env.reset_to_state=np.array([50,50, center[0],center[1], angle])
    obs,info =env.reset()

    obs_deque = collections.deque(
        [obs]*obs_horizon,maxlen=obs_horizon)
    imgs=[env.render(mode='rgb_array')]
    rewards=list()
    done=False
    step_idx=0



    action_list=[]

    with tqdm(total=max_steps,desc="Eval PushTImageEnv")as pbar:
        while not done:
            
            B=1
            
            images = np.stack([x['image'] for x in obs_deque])
            agent_poses = np.stack([x['agent_pos']for x in obs_deque])

            nagent_poses=  normalize_data(agent_poses,stats=stats['agent_pos'])
            nimages=images
            
            nimages=torch.from_numpy(nimages).to(device,dtype=torch.float32)
            nagent_poses=torch.from_numpy(nagent_poses).to(device,dtype=torch.float32)
            
            with torch.no_grad():
                image_features = ema_nets['vision_encoder'](nimages)
                # (2,512)

                # concat with low-dim observations
                obs_features = torch.cat([image_features, nagent_poses], dim=-1)

                # reshape observation to (B,obs_horizon*obs_dim)
                obs_cond = obs_features.unsqueeze(0).flatten(start_dim=1)

                # initialize action from Guassian noise
                noisy_action = torch.randn(
                    (B, pred_horizon, action_dim), device=device)
                naction = noisy_action

                # init scheduler
                noise_scheduler.set_timesteps(num_diffusion_iters)

                for k in noise_scheduler.timesteps:
                    # predict noise
                    noise_pred = ema_nets['noise_pred_net'](
                        sample=naction,
                        timestep=k,
                        global_cond=obs_cond
                    )

                    # inverse diffusion step (remove noise)
                    naction = noise_scheduler.step(
                        model_output=noise_pred,
                        timestep=k,
                        sample=naction
                    ).prev_sample

            # unnormalize action
            naction = naction.detach().to('cpu').numpy()
            # (B, pred_horizon, action_dim)
            naction = naction[0]
            action_pred =  unnormalize_data(naction, stats=stats['action'])

            # only take action_horizon number of actions
            start = obs_horizon - 1
            end = start + action_horizon
            action = action_pred[start:end,:]
            # (action_horizon, action_dim)

            # execute action_horizon number of steps
            # without replanning
            for act in action:
                action_list.append(act)
            
            ##to show trajectory
            # predit_imge=imgs[-1]
            # predit_imge = cv2.resize(predit_imge, dsize=(512, 512), interpolation=cv2.INTER_AREA)
            # for pos in action:
            #     cv2.circle(predit_imge,(int(pos[0]),int(pos[1])),5,(0,0,255),-1)
            # cv2.imshow("pimg",predit_imge)
            # cv2.waitKey(0)

            
            
            for i in range(len(action)):
                # stepping env
                obs, reward, done, _, info = env.step(action[i])
                # save observations
                obs_deque.append(obs)
                # and reward/vis
                rewards.append(reward)
                imgs.append(env.render(mode='rgb_array'))
                # cv2.imshow("imgs",imgs[-1])
                # cv2.waitKey(0)
                # # update progress bar
                step_idx += 1
                pbar.update(1)
                pbar.set_postfix(reward=reward)
                # print(step_idx,"steps")
                if step_idx > max_steps or reward > 0.95:
                    done = True
                if done:
                    break


    # print(action_list)

    # predit_imge=cam_img
    # for pos in action_list:
    #     cv2.circle(predit_imge,(int(pos[0]),int(pos[1])),5,(0,0,255),-1)
    # cv2.imshow("pimg",predit_imge)
    # cv2.waitKey(0)

    # print out the maximum target coverage
    print('Score: ', max(rewards))

    # visualize
    # from IPython.display import Video
    # vwrite('vis.mp4', imgs)

    action_publisher.publish_actions(action_list)

    rclpy.shutdown()

if __name__ == '__main__':
    main()