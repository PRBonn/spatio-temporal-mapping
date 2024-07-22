# MIT License
#
# Copyright (c) 2024 Luca Lobefaro, Meher V.R. Malladi, Tiziano Guadagnino, Cyrill Stachniss
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
from pathlib import Path
from typing import Optional
from st_mapping.datasets import (
    available_dataloaders,
    dataloader_name_callback,
    dataset_factory,
)

import typer

from st_mapping.config.parser import load_config

from st_mapping.deform_ref_pipeline import DeformRefPipeline


app = typer.Typer(no_args_is_help=True, add_completion=False, rich_markup_mode="rich")

docstring = f"""
st_mapping-deform-ref allows you to deform a previously built map of plants basing on a new sequence, even if string deformations occurred.
Two important assumptions are made: 
1. The base of the plants is visible and it does not change position between the two recordings.
2. The second sequence starts at (more or less) the same pose of the first sequence.
"""


@app.command(help=docstring)
def st_mapping_deform_ref(
    dataset_folder: Path = typer.Argument(
        ..., help="Path to the dataset folder.", show_default=False
    ),
    reference_dataset_folder: Path = typer.Argument(
        ..., help="PAth to the reference dataset folder.", show_default=False
    ),
    dataloader: str = typer.Option(
        None,
        show_default=False,
        case_sensitive=False,
        autocompletion=available_dataloaders,
        callback=dataloader_name_callback,
        help="[Optional] Use a specific dataloader from those supported.",
    ),
    config_filename: Optional[Path] = typer.Option(
        None,
        "--config",
        exists=True,
        show_default=False,
        help="[Optional] Path to the configuration file",
    ),
):
    # Argument parsing
    if not dataloader:
        dataloader = "generic"
    config = load_config(config_filename)

    # Initialization
    dataset = dataset_factory(
        dataloader=dataloader,
        dataset_folder=dataset_folder,
        has_poses=False,
        depth_scale=config.dataset.depth_scale,
    )
    ref_dataset = dataset_factory(
        dataloader=dataloader,
        dataset_folder=reference_dataset_folder,
        has_poses=True,
        depth_scale=config.dataset.depth_scale,
    )

    if not ref_dataset.has_map():
        print("Reference dataset must have an associated map.")
        exit(1)

    # Run pipeline
    DeformRefPipeline(
        dataset=dataset,
        ref_dataset=ref_dataset,
        config=config,
    ).run().print()


def run():
    app()
