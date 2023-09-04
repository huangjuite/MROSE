import torch
import torch.nn as nn
import torch.nn.functional as F


class RadarTransformer(nn.Module):
    def __init__(
        self,
        feature_dim=5,
        embed_dim=32,
        n_head=8,
        n_layer=3
    ) -> None:
        super(RadarTransformer, self).__init__()

        self.linear_project = nn.Linear(feature_dim, embed_dim)

        encoder_layer = nn.TransformerEncoderLayer(
            d_model=embed_dim, nhead=n_head)
        self.attention = nn.TransformerEncoder(encoder_layer, num_layers=n_layer)

        self.linear_unproject = nn.Linear(embed_dim, 1)

    def forward(self, x, pad_mask):
        x = self.linear_project(x)
        x = self.attention.forward(
            x, src_key_padding_mask=pad_mask)
        x = self.linear_unproject(x)
        x = torch.sigmoid(x)

        return x
    

        