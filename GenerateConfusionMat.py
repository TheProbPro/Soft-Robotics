import numpy as np
import pandas as pd
from sklearn.metrics import confusion_matrix, classification_report
import matplotlib.pyplot as plt
import matplotlib as mpl

mpl.rcParams['text.usetex'] = True
mpl.rcParams['font.family'] = 'serif'

plt.rc('xtick', labelsize=22)
plt.rc('ytick', labelsize=22)

# Define all possible labels (both what you expressed + what participants could pick)
true_labels = ['Sad', 'Neutral', 'Calm', 'Curious', 'Angry']
labels = ['Sad', 'Neutral', 'Calm', 'Curious', 'Angry', 'Happy', 'Scared']

def compute_confusion_matrix(y_true, y_pred, labels=labels, normalize: bool = False):
    """
    Compute confusion matrix, with consistent rows/columns for all labels.
    """
    cm = confusion_matrix(y_true, y_pred, labels=labels)
    df_cm = pd.DataFrame(cm, index=labels, columns=labels)
    if normalize:
        # Normalize rows (true-label wise)
        df_cm = df_cm.div(df_cm.sum(axis=1).replace(0, 1), axis=0)
        # (replace zero-sums with 1 to avoid division by zero)
    return df_cm

def plot_confusion_matrix(cm_df, title='Confusion Matrix', fmt=None, cmap=plt.cm.Blues):
    """
    Plot confusion matrix (raw or normalized) using matplotlib, with proper axis labels.
    """
    fig, ax = plt.subplots(figsize=(8, 6))
    im = ax.imshow(cm_df.values, interpolation='nearest', cmap=cmap)
    fig.colorbar(im, ax=ax)

    # Tick labels
    ax.set_xticks(np.arange(cm_df.shape[1]))
    ax.set_yticks(np.arange(cm_df.shape[0]))
    ax.set_xticklabels(cm_df.columns, rotation=45, ha='right')
    ax.set_yticklabels(cm_df.index)

    ax.set_ylabel('True label', fontsize=26)
    ax.set_xlabel('Predicted label', fontsize=26)
    ax.set_title(title, fontsize=28)

    # Format each cell
    if fmt is None:
        # Decide format based on data type
        fmt = '.2f' if cm_df.values.dtype.kind == 'f' else 'd'
    thresh = cm_df.values.max() / 2.0
    for i in range(cm_df.shape[0]):
        for j in range(cm_df.shape[1]):
            val = cm_df.values[i, j]
            if not np.isnan(val):
                ax.text(j, i, format(val, fmt),
                        ha='center', va='center',
                        fontsize=22, color='white' if val > thresh else 'black')
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    # Example data
    y_true = [
        'Sad', 'Calm', 'Angry', 'Neutral', 'Curious',
        'Sad', 'Calm', 'Angry', 'Neutral', 'Curious',
        'Sad', 'Calm', 'Angry', 'Neutral', 'Curious',
        'Sad', 'Calm', 'Angry', 'Neutral', 'Curious',
        'Sad', 'Calm', 'Angry', 'Neutral', 'Curious',
        'Sad', 'Calm', 'Angry', 'Neutral', 'Curious',
        'Sad', 'Calm', 'Angry', 'Neutral', 'Curious',
        'Sad', 'Calm', 'Angry', 'Neutral', 'Curious',
        'Sad', 'Calm', 'Angry', 'Neutral', 'Curious',
        'Sad', 'Calm', 'Angry', 'Neutral', 'Curious',
        'Sad', 'Calm', 'Angry', 'Neutral', 'Curious',
        'Sad', 'Calm', 'Angry', 'Neutral', 'Curious',
        'Sad', 'Calm', 'Angry', 'Neutral', 'Curious',
        'Sad', 'Calm', 'Angry', 'Neutral', 'Curious'
    ]
    y_pred = [
        'Neutral', 'Calm', 'Angry', 'Sad', 'Scared',
        'Scared', 'Curious', 'Angry', 'Sad', 'Angry',
        'Neutral', 'Curious', 'Angry', 'Happy', 'Scared',
        'Scared', 'Neutral', 'Angry', 'Calm', 'Curious',
        'Sad', 'Calm', 'Angry', 'Calm', 'Curious',
        'Sad', 'Calm', 'Angry', 'Neutral', 'Sad',
        'Sad', 'Calm', 'Angry', 'Calm', 'Scared',
        'Neutral', 'Sad', 'Angry', 'Calm', 'Neutral',
        'Calm', 'Curious', 'Angry', 'Scared', 'Neutral',
        'Neutral', 'Scared', 'Angry', 'Neutral', 'Happy',
        'Sad', 'Calm', 'Angry', 'Neutral', 'Scared',
        'Sad', 'Neutral', 'Angry', 'Calm', 'Curious',
        'Sad', 'Curious', 'Angry', 'Curious', 'Happy',
        'Sad',  'Calm', 'Angry', 'Sad', 'Scared'
    ]

    # Raw counts
    df_cm = compute_confusion_matrix(y_true, y_pred, labels=labels, normalize=False)
    print("Confusion matrix (counts):")
    print(df_cm)

    # Normalized (fractions per true label)
    df_cm_norm = compute_confusion_matrix(y_true, y_pred, labels=labels, normalize=True)
    print("\nConfusion matrix (normalized):")
    print(df_cm_norm)

    # Asymetric matrix
    df_cm_norm_asym = compute_confusion_matrix(y_true, y_pred, labels=true_labels + [l for l in labels if l not in true_labels], normalize=True)
    print("\nConfusion matrix (normalized):")
    print(df_cm_norm_asym)
    df_cm_norm_asym = pd.DataFrame(df_cm_norm_asym, index=true_labels, columns=labels)
    plot_confusion_matrix(df_cm_norm_asym, title='Expressed Emotions vs. Predicted Emotions')

    # Plot
    #plot_confusion_matrix(df_cm_norm, title='Normalized Confusion Matrix')
