import DQN_cart_pole as cart_pole
import torch

num_episodes = 50
for i_episode in range(num_episodes):
    # Initialize the environment and state
    cart_pole.env.reset()
    last_screen = cart_pole.get_screen()
    current_screen = cart_pole.get_screen()
    state = current_screen - last_screen
    for t in cart_pole.count():
        # Select and perform an action
        action = cart_pole.select_action(state)
        _, reward, done, _ = cart_pole.env.step(action.item())
        reward = torch.tensor([reward], device=cart_pole.device)

        # Observe new state
        last_screen = current_screen
        current_screen = cart_pole.get_screen()
        if not done:
            next_state = current_screen - last_screen
        else:
            next_state = None

        # Store the transition in memory
        cart_pole.memory.push(state, action, next_state, reward)

        # Move to the next state
        state = next_state

        # Perform one step of the optimization (on the target network)
        cart_pole.optimize_model()
        if done:
            cart_pole.episode_durations.append(t + 1)
            cart_pole.plot_durations()
            break
    # Update the target network, copying all weights and biases in DQN
    if i_episode % cart_pole.TARGET_UPDATE == 0:
        cart_pole.target_net.load_state_dict(cart_pole.policy_net.state_dict())

print('Complete')
cart_pole.env.render()
cart_pole.env.close()
cart_pole.plt.ioff()
cart_pole.plt.show()