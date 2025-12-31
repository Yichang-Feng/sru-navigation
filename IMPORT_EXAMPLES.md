# SRU PyTorch - Import Examples

After installing the package with `pip install sru-pytorch`, you can import and use the three SRU networks as shown below.

> **📦 Pip Package Contents**: The pip-installable package includes three SRU networks: `LSTM_SRU`, `LSTM_SRU_Gate`, and `GRU_SRU`.

> **📌 Note**: MambaNet and S4 are baseline implementations included in the repository for experimental comparisons, but are **not part of the pip package**.

## Installation

### Standard Installation

```bash
pip install sru-pytorch
```

This installs the three SRU networks: **LSTM_SRU**, **LSTM_SRU_Gate**, and **GRU_SRU**.

### Development Installation

For development or to access baseline implementations (MambaNet, S4):

```bash
git clone <repository-url>
cd sru-pytorch-spatial-learning
pip install -e .                    # Installs three SRU networks
```

## Import Methods

### Method 1: Import from network module (Recommended)

```python
import torch
from network import LSTM_SRU, LSTM_SRU_Gate, GRU_SRU

# Create models (all three SRU networks)
lstm_sru = LSTM_SRU(input_size=15, hidden_size=128, num_layers=2)
lstm_sru_gate = LSTM_SRU_Gate(input_size=15, hidden_size=128, num_layers=2)
gru_sru = GRU_SRU(input_size=15, hidden_size=128, num_layers=2)
```

### Method 2: Import specific classes

```python
import torch
from network import LSTM_SRU, LSTMSRUCell
from network import LSTM_SRU_Gate, LSTMSRUGateCell
from network import GRU_SRU, GRUSRUCell

# Use the classes
model = LSTM_SRU(input_size=15, hidden_size=128)
```

---

## Detailed Usage Examples

### LSTM_SRU: LSTM with Spatial Transformation Gates

```python
import torch
from network import LSTM_SRU

# Create model
batch_size = 4
input_size = 15
hidden_size = 128
seq_len = 10

model = LSTM_SRU(
    input_size=input_size,
    hidden_size=hidden_size,
    num_layers=2,
    batch_first=True  # Set True for (batch, seq_len, features) format
)

# Prepare input
x = torch.randn(batch_size, seq_len, input_size)

# Forward pass
output, (h_n, c_n) = model(x)

print(f"Output shape: {output.shape}")           # (batch_size, seq_len, hidden_size)
print(f"Hidden state shape: {h_n.shape}")        # (num_layers, batch_size, hidden_size)
print(f"Cell state shape: {c_n.shape}")          # (num_layers, batch_size, hidden_size)
```

**Key Features:**
- Additive transformation gates for spatial learning
- LSTM variant with enhanced spatial memory
- Supports multi-layer architecture
- Compatible with batch-first and time-first formats

---

### LSTM_SRU_Gate: LSTM with Additive Transformation and Gated Refinement

```python
import torch
from network import LSTM_SRU_Gate

# Create model
batch_size = 4
input_size = 15
hidden_size = 128
seq_len = 10

model = LSTM_SRU_Gate(
    input_size=input_size,
    hidden_size=hidden_size,
    num_layers=2,
    batch_first=True  # Set True for (batch, seq_len, features) format
)

# Prepare input
x = torch.randn(batch_size, seq_len, input_size)

# Forward pass
output, (h_n, c_n) = model(x)

print(f"Output shape: {output.shape}")           # (batch_size, seq_len, hidden_size)
print(f"Hidden state shape: {h_n.shape}")        # (num_layers, batch_size, hidden_size)
print(f"Cell state shape: {c_n.shape}")          # (num_layers, batch_size, hidden_size)
```

**Key Features:**
- Additive transformation with gated refinement mechanism
- Enhanced control over spatial transformation process
- LSTM variant with both cell and hidden states
- Supports multi-layer architecture
- More sophisticated gating for complex spatial tasks

---

### GRU_SRU: GRU with Spatial Transformation Gates

```python
import torch
from network import GRU_SRU

# Create model
batch_size = 4
input_size = 15
hidden_size = 128
seq_len = 10

model = GRU_SRU(
    input_size=input_size,
    hidden_size=hidden_size,
    num_layers=2,
    batch_first=True  # Set True for (batch, seq_len, features) format
)

# Prepare input
x = torch.randn(batch_size, seq_len, input_size)

# Forward pass
output, (h_n, _) = model(x)

print(f"Output shape: {output.shape}")           # (batch_size, seq_len, hidden_size)
print(f"Hidden state shape: {h_n.shape}")        # (num_layers, batch_size, hidden_size)
```

**Key Features:**
- Additive transformation gates for spatial learning
- GRU variant with enhanced spatial memory
- Lighter than LSTM (single hidden state instead of separate cell state)
- Supports multi-layer architecture
- Faster training and inference with fewer parameters

---

## Complete Training Example

```python
import torch
import torch.nn as nn
from network import LSTM_SRU, LSTM_SRU_Gate, GRU_SRU

# Hyperparameters
batch_size = 32
input_size = 15
hidden_size = 256
num_layers = 3
seq_len = 20
output_size = 10
learning_rate = 1e-3
epochs = 100

# Create model (choose one of the three SRU networks)
model = LSTM_SRU(
    input_size=input_size,
    hidden_size=hidden_size,
    num_layers=num_layers,
    batch_first=True
)

# Or use LSTM_SRU_Gate for gated refinement
# model = LSTM_SRU_Gate(input_size=input_size, hidden_size=hidden_size, num_layers=num_layers, batch_first=True)

# Or use GRU_SRU for lighter alternative
# model = GRU_SRU(input_size=input_size, hidden_size=hidden_size, num_layers=num_layers, batch_first=True)

# Loss and optimizer
loss_fn = nn.MSELoss()
optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)

# Training loop
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
model = model.to(device)

for epoch in range(epochs):
    # Dummy data (replace with your data)
    x = torch.randn(batch_size, seq_len, input_size).to(device)
    y = torch.randn(batch_size, output_size).to(device)

    # Forward pass
    output, _ = model(x)
    # Use last timestep for prediction (or average, or attention)
    predictions = output[:, -1, :]  # Take last time step

    # Backward pass
    loss = loss_fn(predictions, y)
    optimizer.zero_grad()
    loss.backward()
    optimizer.step()

    if (epoch + 1) % 10 == 0:
        print(f"Epoch [{epoch+1}/{epochs}], Loss: {loss.item():.4f}")
```

---

## Device Management

```python
import torch
from network import LSTM_SRU

# Check if GPU is available
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
print(f"Using device: {device}")

# Create and move model to device
model = LSTM_SRU(input_size=15, hidden_size=128)
model = model.to(device)

# Prepare data on the same device
x = torch.randn(4, 10, 15).to(device)

# Forward pass
output, state = model(x)
```

---

## Comparing the Three SRU Networks

```python
import torch
from network import LSTM_SRU, LSTM_SRU_Gate, GRU_SRU

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

# Input data
batch_size, seq_len, input_size = 4, 10, 15

# Create all three SRU models
models = {
    'LSTM_SRU': LSTM_SRU(input_size=input_size, hidden_size=128, num_layers=2, batch_first=True),
    'LSTM_SRU_Gate': LSTM_SRU_Gate(input_size=input_size, hidden_size=128, num_layers=2, batch_first=True),
    'GRU_SRU': GRU_SRU(input_size=input_size, hidden_size=128, num_layers=2, batch_first=True),
}

x = torch.randn(batch_size, seq_len, input_size).to(device)

for name, model in models.items():
    model = model.to(device)
    model.eval()

    with torch.no_grad():
        output, _ = model(x)
        print(f"{name}:")
        print(f"  Output shape: {output.shape}")
        print(f"  Total parameters: {sum(p.numel() for p in model.parameters())}")
```

---

## Use Cases

### LSTM_SRU vs LSTM_SRU_Gate vs GRU_SRU

- **LSTM_SRU**: Best for tasks requiring long-term dependencies with separate memory cells. Good baseline for spatial-temporal learning with standard LSTM architecture enhanced by spatial transformation gates.

- **LSTM_SRU_Gate**: Enhanced version with gated refinement mechanism. Provides more control over the spatial transformation process. Best for complex spatial tasks requiring sophisticated gating mechanisms.

- **GRU_SRU**: Lighter alternative to LSTM-based variants with comparable performance. Faster training and inference with fewer parameters. Ideal for applications where computational efficiency is important.

---

## Citation

If you use these models in your research, please cite the paper:

```bibtex
@article{yang2025sru,
  author = {Yang, Fan and Frivik, Per and Hoeller, David and Wang, Chen and Cadena, Cesar and Hutter, Marco},
  title = {Spatially-enhanced recurrent memory for long-range mapless navigation via end-to-end reinforcement learning},
  journal = {The International Journal of Robotics Research},
  year = {2025},
  doi = {10.1177/02783649251401926},
  url = {https://doi.org/10.1177/02783649251401926}
}
```
