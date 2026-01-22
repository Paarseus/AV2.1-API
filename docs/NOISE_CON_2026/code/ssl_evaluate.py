#!/usr/bin/env python3
"""
SSL Evaluation Script for Road Surface Classification

Evaluates trained models and generates figures for the paper.

Usage:
    python ssl_evaluate.py --model ../models/finetuned/finetuned.pt --data ../data/processed/ --output ../figures/

Features:
- Detailed classification metrics (accuracy, precision, recall, F1)
- Confusion matrix
- t-SNE embedding visualization
- Per-class analysis
"""

import os
import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from collections import defaultdict

import torch
import torch.fft as fft
from torch.utils.data import DataLoader
from sklearn.metrics import (
    classification_report, confusion_matrix, accuracy_score,
    precision_score, recall_score, f1_score
)
from sklearn.manifold import TSNE

# Add TFC to path
TFC_PATH = Path(__file__).parent / 'TFC-pretraining' / 'code' / 'TFC'
sys.path.insert(0, str(TFC_PATH))

from model import TFC, target_classifier

# Add parent for ssl_train
sys.path.insert(0, str(Path(__file__).parent))
from ssl_train import RoadVibrationDataset, get_config

# Label names for road surfaces
LABEL_NAMES = {
    0: 'Asphalt',
    1: 'Sidewalk',
    2: 'Grass',
    3: 'Bump'
}


class ModelEvaluator:
    """Evaluate SSL models for road surface classification."""

    def __init__(self, model_path: str, config=None, device='cuda'):
        self.device = torch.device(device if torch.cuda.is_available() else 'cpu')

        # Load config
        if config is None:
            config = get_config()
        self.config = config

        # Initialize and load model
        self.model = TFC(config).to(self.device)
        self.classifier = target_classifier(config).to(self.device)

        checkpoint = torch.load(model_path, map_location=self.device)
        self.model.load_state_dict(checkpoint['model_state_dict'])
        self.classifier.load_state_dict(checkpoint['classifier_state_dict'])

        self.model.eval()
        self.classifier.eval()

        print(f"Loaded model from {model_path}")

    def extract_embeddings(self, loader):
        """Extract embeddings from data loader."""
        embeddings = []
        labels = []
        predictions = []

        with torch.no_grad():
            for data, label, _, data_f, _ in loader:
                data = data.float().to(self.device)
                data_f = data_f.float().to(self.device)

                # Get embeddings
                h_t, z_t, h_f, z_f = self.model(data, data_f)
                fea_concat = torch.cat((z_t, z_f), dim=1)

                # Get predictions
                preds = self.classifier(fea_concat)

                embeddings.append(fea_concat.cpu().numpy())
                labels.append(label.numpy())
                predictions.append(preds.argmax(dim=1).cpu().numpy())

        embeddings = np.concatenate(embeddings, axis=0)
        labels = np.concatenate(labels, axis=0)
        predictions = np.concatenate(predictions, axis=0)

        return embeddings, labels, predictions

    def evaluate(self, loader):
        """Full evaluation with detailed metrics."""
        embeddings, labels, predictions = self.extract_embeddings(loader)

        # Filter out unlabeled samples
        mask = labels >= 0
        labels = labels[mask]
        predictions = predictions[mask]
        embeddings = embeddings[mask]

        if len(labels) == 0:
            print("No labeled samples found!")
            return None

        # Get unique labels present in both true and predicted
        all_labels = np.union1d(labels, predictions)
        target_names = [LABEL_NAMES.get(i, f'Class {i}') for i in all_labels]

        # Compute metrics
        results = {
            'accuracy': accuracy_score(labels, predictions) * 100,
            'precision': precision_score(labels, predictions, average='macro', zero_division=0) * 100,
            'recall': recall_score(labels, predictions, average='macro', zero_division=0) * 100,
            'f1': f1_score(labels, predictions, average='macro', zero_division=0) * 100,
            'confusion_matrix': confusion_matrix(labels, predictions, labels=all_labels),
            'classification_report': classification_report(
                labels, predictions,
                labels=all_labels,
                target_names=target_names,
                zero_division=0
            ),
            'embeddings': embeddings,
            'labels': labels,
            'predictions': predictions,
        }

        # Per-class accuracy
        per_class_acc = {}
        for cls in np.unique(labels):
            mask = labels == cls
            cls_acc = accuracy_score(labels[mask], predictions[mask]) * 100
            per_class_acc[LABEL_NAMES.get(cls, f'Class {cls}')] = cls_acc
        results['per_class_accuracy'] = per_class_acc

        return results

    def print_results(self, results):
        """Print evaluation results."""
        print("\n" + "="*60)
        print("EVALUATION RESULTS")
        print("="*60)

        print(f"\nOverall Metrics:")
        print(f"  Accuracy:  {results['accuracy']:.2f}%")
        print(f"  Precision: {results['precision']:.2f}%")
        print(f"  Recall:    {results['recall']:.2f}%")
        print(f"  F1 Score:  {results['f1']:.2f}%")

        print(f"\nPer-Class Accuracy:")
        for cls, acc in results['per_class_accuracy'].items():
            print(f"  {cls}: {acc:.2f}%")

        print(f"\nClassification Report:")
        print(results['classification_report'])

        print(f"\nConfusion Matrix:")
        print(results['confusion_matrix'])

    def plot_confusion_matrix(self, results, output_path: str = None):
        """Plot confusion matrix."""
        cm = results['confusion_matrix']
        # Use union of true and predicted labels to match confusion matrix dimensions
        all_labels = np.union1d(results['labels'], results['predictions'])
        class_names = [LABEL_NAMES.get(i, f'Class {i}') for i in all_labels]

        fig, ax = plt.subplots(figsize=(8, 6))

        # Plot heatmap
        im = ax.imshow(cm, interpolation='nearest', cmap=plt.cm.Blues)
        ax.figure.colorbar(im, ax=ax)

        # Labels
        ax.set(xticks=np.arange(cm.shape[1]),
               yticks=np.arange(cm.shape[0]),
               xticklabels=class_names,
               yticklabels=class_names,
               xlabel='Predicted Label',
               ylabel='True Label',
               title='Confusion Matrix - Road Surface Classification')

        # Rotate x labels
        plt.setp(ax.get_xticklabels(), rotation=45, ha='right', rotation_mode='anchor')

        # Add text annotations
        thresh = cm.max() / 2.
        for i in range(cm.shape[0]):
            for j in range(cm.shape[1]):
                ax.text(j, i, format(cm[i, j], 'd'),
                        ha='center', va='center',
                        color='white' if cm[i, j] > thresh else 'black')

        fig.tight_layout()

        if output_path:
            plt.savefig(output_path, dpi=300, bbox_inches='tight')
            print(f"Saved confusion matrix to {output_path}")

        return fig

    def plot_tsne(self, results, output_path: str = None, perplexity: int = 30):
        """Plot t-SNE visualization of embeddings."""
        embeddings = results['embeddings']
        labels = results['labels']

        if len(embeddings) < 5:
            print(f"Too few samples ({len(embeddings)}) for t-SNE visualization, skipping...")
            return None

        print(f"Computing t-SNE for {len(embeddings)} samples...")

        # Compute t-SNE with safe perplexity
        safe_perplexity = min(perplexity, max(1, len(embeddings) // 2))
        tsne = TSNE(n_components=2, perplexity=safe_perplexity,
                    random_state=42, n_iter=1000)
        embeddings_2d = tsne.fit_transform(embeddings)

        # Plot
        fig, ax = plt.subplots(figsize=(10, 8))

        unique_labels = sorted(np.unique(labels))
        colors = plt.cm.Set1(np.linspace(0, 1, len(unique_labels)))

        for idx, cls in enumerate(unique_labels):
            mask = labels == cls
            ax.scatter(embeddings_2d[mask, 0], embeddings_2d[mask, 1],
                       c=[colors[idx]], label=LABEL_NAMES.get(cls, f'Class {cls}'),
                       alpha=0.7, s=50)

        ax.set_xlabel('t-SNE Dimension 1')
        ax.set_ylabel('t-SNE Dimension 2')
        ax.set_title('t-SNE Visualization of Learned Representations')
        ax.legend(loc='best')
        ax.grid(True, alpha=0.3)

        fig.tight_layout()

        if output_path:
            plt.savefig(output_path, dpi=300, bbox_inches='tight')
            print(f"Saved t-SNE plot to {output_path}")

        return fig

    def compare_with_baseline(self, results, baseline_results: dict = None):
        """Compare SSL results with baseline (e.g., random forest on raw features)."""
        print("\n" + "="*60)
        print("COMPARISON WITH BASELINE")
        print("="*60)

        if baseline_results:
            print(f"\n{'Metric':<12} {'SSL':>10} {'Baseline':>10} {'Delta':>10}")
            print("-"*45)
            for metric in ['accuracy', 'precision', 'recall', 'f1']:
                ssl_val = results[metric]
                base_val = baseline_results.get(metric, 0)
                delta = ssl_val - base_val
                print(f"{metric.capitalize():<12} {ssl_val:>9.2f}% {base_val:>9.2f}% {delta:>+9.2f}%")
        else:
            print("No baseline results provided for comparison.")

    def export_latex_table(self, results, output_path: str):
        """Export results as LaTeX table for paper."""
        latex = r"""\begin{table}[htbp]
\centering
\caption{Road Surface Classification Results}
\label{tab:ssl_results}
\begin{tabular}{lc}
\toprule
\textbf{Metric} & \textbf{Value (\%%)} \\
\midrule
Accuracy & %.2f \\
Precision & %.2f \\
Recall & %.2f \\
F1 Score & %.2f \\
\midrule
\multicolumn{2}{c}{\textbf{Per-Class Accuracy}} \\
\midrule
""" % (results['accuracy'], results['precision'], results['recall'], results['f1'])

        for cls, acc in results['per_class_accuracy'].items():
            latex += f"{cls} & {acc:.2f} \\\\\n"

        latex += r"""\bottomrule
\end{tabular}
\end{table}
"""

        with open(output_path, 'w') as f:
            f.write(latex)
        print(f"Saved LaTeX table to {output_path}")


def main():
    parser = argparse.ArgumentParser(description='Evaluate SSL model')
    parser.add_argument('--model', required=True, help='Path to trained model')
    parser.add_argument('--data', required=True, help='Path to test data directory')
    parser.add_argument('--output', default='../figures/', help='Output directory for figures')
    parser.add_argument('--device', default='cuda', help='Device (cuda/cpu)')
    parser.add_argument('--no-plots', action='store_true', help='Skip plotting')
    args = parser.parse_args()

    # Setup paths
    data_path = Path(args.data)
    output_path = Path(args.output)
    output_path.mkdir(parents=True, exist_ok=True)

    # Load config and evaluator
    config = get_config()
    evaluator = ModelEvaluator(args.model, config, args.device)

    # Load test data
    test_file = data_path / 'test.pt' if data_path.is_dir() else data_path
    test_dataset = RoadVibrationDataset(str(test_file), config, 'fine_tune')
    test_loader = DataLoader(
        test_dataset,
        batch_size=config.target_batch_size,
        shuffle=False,
        drop_last=False,
        num_workers=0
    )

    # Evaluate
    results = evaluator.evaluate(test_loader)

    if results is None:
        print("Evaluation failed - no labeled samples found.")
        return

    # Print results
    evaluator.print_results(results)

    # Generate plots
    if not args.no_plots:
        evaluator.plot_confusion_matrix(results, str(output_path / 'confusion_matrix.png'))
        evaluator.plot_tsne(results, str(output_path / 'tsne_embeddings.png'))

    # Export LaTeX table
    evaluator.export_latex_table(results, str(output_path / 'results_table.tex'))

    # Save raw results
    torch.save({
        'accuracy': results['accuracy'],
        'precision': results['precision'],
        'recall': results['recall'],
        'f1': results['f1'],
        'per_class_accuracy': results['per_class_accuracy'],
        'confusion_matrix': results['confusion_matrix'],
    }, str(output_path / 'evaluation_results.pt'))

    print(f"\nAll results saved to {output_path}")


if __name__ == '__main__':
    main()
