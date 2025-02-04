import torch
import torch.nn as nn
from torch.utils.data import DataLoader, random_split
from dataset import ViNTDataset
from models.vint_model import ViNT  # Ensure this import matches your project structure

def train():
    # Configurations
    root_dir = "path/to/trajectories"
    batch_size = 32
    context_size = 5
    len_traj_pred = 5
    learn_angle = True
    num_epochs = 10
    lr = 1e-4
    train_split = 0.8

    # Dataset and DataLoader
    dataset = ViNTDataset(root_dir, context_size=context_size, len_traj_pred=len_traj_pred)
    train_size = int(train_split * len(dataset))
    val_size = len(dataset) - train_size
    train_dataset, val_dataset = random_split(dataset, [train_size, val_size])
    
    train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True, num_workers=4, pin_memory=True)
    val_loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=False, num_workers=4, pin_memory=True)

    # Model setup
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = ViNT(
        context_size=context_size,
        len_traj_pred=len_traj_pred,
        learn_angle=learn_angle,
        obs_encoder="efficientnet-b0",
        obs_encoding_size=512,
        late_fusion=False,
        mha_num_attention_heads=2,
        mha_num_attention_layers=2,
        mha_ff_dim_factor=4
    ).to(device)

    # Loss and optimizer
    criterion_dist = nn.MSELoss()
    criterion_action = nn.MSELoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)

    # Training loop
    for epoch in range(num_epochs):
        model.train()
        train_loss = 0.0
        for batch, ((obs_img, cmd), (dist_tgt, act_tgt)) in enumerate(train_loader):
            obs_img, cmd = obs_img.to(device), cmd.to(device)
            dist_tgt, act_tgt = dist_tgt.to(device), act_tgt.to(device)

            optimizer.zero_grad()
            dist_pred, act_pred = model(obs_img, cmd)
            
            loss_dist = criterion_dist(dist_pred.squeeze(), dist_tgt)
            loss_act = criterion_action(act_pred, act_tgt)
            loss = loss_dist + loss_act
            loss.backward()
            optimizer.step()

            train_loss += loss.item()
            if batch % 10 == 0:
                print(f"Epoch {epoch}, Batch {batch}, Loss: {loss.item()}")

        # Validation
        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for (obs_img, cmd), (dist_tgt, act_tgt) in val_loader:
                obs_img, cmd = obs_img.to(device), cmd.to(device)
                dist_tgt, act_tgt = dist_tgt.to(device), act_tgt.to(device)
                dist_pred, act_pred = model(obs_img, cmd)
                loss = criterion_dist(dist_pred.squeeze(), dist_tgt) + criterion_action(act_pred, act_tgt)
                val_loss += loss.item()

        print(f"Epoch {epoch}, Train Loss: {train_loss/len(train_loader)}, Val Loss: {val_loss/len(val_loader)}")

    torch.save(model.state_dict(), "vint_model.pth")

if __name__ == "__main__":
    train()