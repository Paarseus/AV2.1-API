#!/usr/bin/env python3
"""
SSL Training Script for Road Surface Classification

Wrapper around TFC framework for self-supervised pretraining on vibration data.

Usage:
    # Pretrain on unlabeled data
    python ssl_train.py --mode pretrain --data ../data/processed/ --output ../models/pretrained/

    # Fine-tune on labeled data
    python ssl_train.py --mode finetune --data ../data/processed/ --pretrained ../models/pretrained/ --output ../models/finetuned/
"""

import os
import sys
import argparse
import numpy as np
from datetime import datetime
from pathlib import Path

import torch
import torch.nn as nn
import torch.fft as fft
from torch.utils.data import Dataset, DataLoader

# Add TFC to path
TFC_PATH = Path(__file__).parent / 'TFC-pretraining' / 'code' / 'TFC'
sys.path.insert(0, str(TFC_PATH))

from model import TFC, target_classifier
from augmentations import DataTransform_FD, DataTransform_TD
from loss import NTXentLoss_poly


class RoadVibrationDataset(Dataset):
    """Dataset for road vibration SSL training."""

    def __init__(self, data_path: str, config, training_mode: str = 'pre_train'):
        """
        Args:
            data_path: Path to .pt file with 'samples' and 'labels'
            config: Config object with hyperparameters
            training_mode: 'pre_train' or 'fine_tune'
        """
        self.training_mode = training_mode

        # Load data
        data = torch.load(data_path)
        self.x_data = data['samples'].float()  # (N, C, T)
        self.y_data = data['labels'].long()

        # Ensure correct shape
        if len(self.x_data.shape) < 3:
            self.x_data = self.x_data.unsqueeze(1)

        # Align time series length
        ts_len = min(self.x_data.shape[-1], config.TSlength_aligned)
        self.x_data = self.x_data[:, :1, :ts_len]  # First channel, aligned length

        # Compute frequency domain representation
        self.x_data_f = fft.fft(self.x_data).abs()

        # Augmentations for pretraining
        if training_mode == 'pre_train':
            self.aug1 = DataTransform_TD(self.x_data, config)
            self.aug1_f = DataTransform_FD(self.x_data_f, config)

        self.len = len(self.x_data)
        print(f"Loaded {self.len} samples, shape: {self.x_data.shape}")

    def __getitem__(self, index):
        if self.training_mode == 'pre_train':
            return (self.x_data[index], self.y_data[index], self.aug1[index],
                    self.x_data_f[index], self.aug1_f[index])
        else:
            return (self.x_data[index], self.y_data[index], self.x_data[index],
                    self.x_data_f[index], self.x_data_f[index])

    def __len__(self):
        return self.len


class SSLTrainer:
    """Self-supervised learning trainer for road vibration data."""

    def __init__(self, config, device='cuda'):
        self.config = config
        self.device = torch.device(device if torch.cuda.is_available() else 'cpu')
        print(f"Using device: {self.device}")

        # Initialize models
        self.model = TFC(config).to(self.device)
        self.classifier = target_classifier(config).to(self.device)

        # Optimizers
        self.model_optimizer = torch.optim.Adam(
            self.model.parameters(),
            lr=config.lr,
            betas=(config.beta1, config.beta2),
            weight_decay=3e-4
        )
        self.classifier_optimizer = torch.optim.Adam(
            self.classifier.parameters(),
            lr=config.lr,
            betas=(config.beta1, config.beta2),
            weight_decay=3e-4
        )

        self.criterion = nn.CrossEntropyLoss()

    def pretrain(self, train_loader, num_epochs=None):
        """Self-supervised pretraining."""
        if num_epochs is None:
            num_epochs = self.config.num_epoch

        print(f"\nStarting pretraining for {num_epochs} epochs...")
        self.model.train()

        for epoch in range(1, num_epochs + 1):
            total_loss = []

            for batch_idx, (data, labels, aug1, data_f, aug1_f) in enumerate(train_loader):
                data = data.float().to(self.device)
                aug1 = aug1.float().to(self.device)
                data_f = data_f.float().to(self.device)
                aug1_f = aug1_f.float().to(self.device)

                self.model_optimizer.zero_grad()

                # Forward pass
                h_t, z_t, h_f, z_f = self.model(data, data_f)
                h_t_aug, z_t_aug, h_f_aug, z_f_aug = self.model(aug1, aug1_f)

                # Compute contrastive loss
                nt_xent = NTXentLoss_poly(
                    self.device,
                    data.shape[0],  # batch size
                    self.config.Context_Cont.temperature,
                    self.config.Context_Cont.use_cosine_similarity
                )

                loss_t = nt_xent(h_t, h_t_aug)  # Time domain consistency
                loss_f = nt_xent(h_f, h_f_aug)  # Frequency domain consistency
                loss_tf = nt_xent(z_t, z_f)    # Time-frequency consistency

                # Combined loss
                lam = 0.2
                loss = lam * (loss_t + loss_f) + loss_tf

                loss.backward()
                self.model_optimizer.step()

                total_loss.append(loss.item())

            avg_loss = np.mean(total_loss)
            print(f"Epoch {epoch}/{num_epochs} - Loss: {avg_loss:.4f}")

        print("Pretraining complete!")

    def finetune(self, train_loader, val_loader, num_epochs=None):
        """Supervised fine-tuning on labeled data."""
        if num_epochs is None:
            num_epochs = self.config.num_epoch

        print(f"\nStarting fine-tuning for {num_epochs} epochs...")
        best_acc = 0.0

        for epoch in range(1, num_epochs + 1):
            # Training
            self.model.train()
            self.classifier.train()
            train_loss = []
            train_correct = 0
            train_total = 0

            for data, labels, aug1, data_f, aug1_f in train_loader:
                data = data.float().to(self.device)
                labels = labels.long().to(self.device)
                data_f = data_f.float().to(self.device)

                self.model_optimizer.zero_grad()
                self.classifier_optimizer.zero_grad()

                # Forward
                h_t, z_t, h_f, z_f = self.model(data, data_f)
                fea_concat = torch.cat((z_t, z_f), dim=1)
                predictions = self.classifier(fea_concat)

                loss = self.criterion(predictions, labels)
                loss.backward()

                self.model_optimizer.step()
                self.classifier_optimizer.step()

                train_loss.append(loss.item())
                pred = predictions.argmax(dim=1)
                train_correct += pred.eq(labels).sum().item()
                train_total += labels.size(0)

            train_acc = 100.0 * train_correct / train_total
            avg_train_loss = np.mean(train_loss)

            # Validation
            val_acc, val_loss = self.evaluate(val_loader)

            print(f"Epoch {epoch}/{num_epochs} - "
                  f"Train Loss: {avg_train_loss:.4f}, Train Acc: {train_acc:.2f}% | "
                  f"Val Loss: {val_loss:.4f}, Val Acc: {val_acc:.2f}%")

            if val_acc > best_acc:
                best_acc = val_acc
                self.best_model_state = {
                    'model': self.model.state_dict(),
                    'classifier': self.classifier.state_dict(),
                    'epoch': epoch,
                    'val_acc': val_acc
                }

        print(f"Fine-tuning complete! Best val accuracy: {best_acc:.2f}%")

    def evaluate(self, loader):
        """Evaluate model on data loader."""
        self.model.eval()
        self.classifier.eval()

        total_loss = []
        correct = 0
        total = 0

        with torch.no_grad():
            for data, labels, _, data_f, _ in loader:
                data = data.float().to(self.device)
                labels = labels.long().to(self.device)
                data_f = data_f.float().to(self.device)

                h_t, z_t, h_f, z_f = self.model(data, data_f)
                fea_concat = torch.cat((z_t, z_f), dim=1)
                predictions = self.classifier(fea_concat)

                loss = self.criterion(predictions, labels)
                total_loss.append(loss.item())

                pred = predictions.argmax(dim=1)
                correct += pred.eq(labels).sum().item()
                total += labels.size(0)

        acc = 100.0 * correct / total if total > 0 else 0.0
        avg_loss = np.mean(total_loss) if total_loss else 0.0
        return acc, avg_loss

    def save(self, output_dir: str, name: str = 'model'):
        """Save model checkpoint."""
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)

        checkpoint = {
            'model_state_dict': self.model.state_dict(),
            'classifier_state_dict': self.classifier.state_dict(),
        }

        path = output_path / f'{name}.pt'
        torch.save(checkpoint, path)
        print(f"Model saved to {path}")

    def load(self, checkpoint_path: str, load_classifier: bool = True):
        """Load model from checkpoint."""
        checkpoint = torch.load(checkpoint_path, map_location=self.device, weights_only=True)
        self.model.load_state_dict(checkpoint['model_state_dict'])

        if load_classifier and 'classifier_state_dict' in checkpoint:
            self.classifier.load_state_dict(checkpoint['classifier_state_dict'])

        print(f"Loaded model from {checkpoint_path}")


def get_config():
    """Load road vibration configuration."""
    # Import from TFC config
    config_path = TFC_PATH.parent / 'config_files'
    sys.path.insert(0, str(config_path))
    from RoadVibration_Configs import Config
    return Config()


def main():
    parser = argparse.ArgumentParser(description='SSL Training for Road Vibration')
    parser.add_argument('--mode', choices=['pretrain', 'finetune'], required=True,
                        help='Training mode')
    parser.add_argument('--data', required=True, help='Data directory or file')
    parser.add_argument('--output', required=True, help='Output directory')
    parser.add_argument('--pretrained', default=None,
                        help='Pretrained model path (for finetune mode)')
    parser.add_argument('--epochs', type=int, default=None,
                        help='Number of epochs (default: from config)')
    parser.add_argument('--batch-size', type=int, default=None,
                        help='Batch size (default: from config)')
    parser.add_argument('--device', default='cuda', help='Device (cuda/cpu)')
    args = parser.parse_args()

    # Load config
    config = get_config()
    if args.batch_size:
        config.batch_size = args.batch_size
        config.target_batch_size = args.batch_size
    if args.epochs:
        config.num_epoch = args.epochs

    # Setup paths
    data_path = Path(args.data)
    output_path = Path(args.output)

    # Initialize trainer
    trainer = SSLTrainer(config, args.device)

    if args.mode == 'pretrain':
        # Load training data
        train_file = data_path / 'train.pt' if data_path.is_dir() else data_path
        train_dataset = RoadVibrationDataset(str(train_file), config, 'pre_train')
        train_loader = DataLoader(
            train_dataset,
            batch_size=config.batch_size,
            shuffle=True,
            drop_last=True,
            num_workers=0
        )

        # Pretrain
        trainer.pretrain(train_loader)
        trainer.save(args.output, 'pretrained')

    elif args.mode == 'finetune':
        # Load pretrained model
        if args.pretrained:
            pretrain_path = Path(args.pretrained)
            if pretrain_path.is_dir():
                pretrain_path = pretrain_path / 'pretrained.pt'
            trainer.load(str(pretrain_path), load_classifier=False)

        # Load train and val data
        train_file = data_path / 'train.pt' if data_path.is_dir() else data_path
        val_file = data_path / 'val.pt' if data_path.is_dir() else data_path

        train_dataset = RoadVibrationDataset(str(train_file), config, 'fine_tune')
        val_dataset = RoadVibrationDataset(str(val_file), config, 'fine_tune')

        train_loader = DataLoader(
            train_dataset,
            batch_size=config.target_batch_size,
            shuffle=True,
            drop_last=True,
            num_workers=0
        )
        val_loader = DataLoader(
            val_dataset,
            batch_size=config.target_batch_size,
            shuffle=False,
            drop_last=False,
            num_workers=0
        )

        # Fine-tune
        trainer.finetune(train_loader, val_loader)
        trainer.save(args.output, 'finetuned')

    print(f"\nTraining complete! Output saved to {args.output}")


if __name__ == '__main__':
    main()
