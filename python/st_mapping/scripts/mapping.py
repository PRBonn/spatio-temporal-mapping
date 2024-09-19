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

from st_mapping.mapping_pipeline import MappingPipeline


app = typer.Typer(no_args_is_help=True, add_completion=False, rich_markup_mode="rich")


@app.command()
def st_mapping_mapping(
    dataset_folder: Path = typer.Argument(
        ..., help="Path to the dataset folder.", show_default=False
    ),
    dataloader: str = typer.Option(
        None,
        show_default=False,
        case_sensitive=False,
        autocompletion=available_dataloaders,
        callback=dataloader_name_callback,
        help="[Optional] Use a specific dataloader from those supported.",
    ),
    just_mapping: bool = typer.Option(
        False,
        "--just-mapping",
        show_default=True,
        help="[Optional] If you want to perform just mapping with given poses (in this case the folder should contain a pose.txt file) or the actual visual odometry (default behaviour)",
    ),
    config_filename: Optional[Path] = typer.Option(
        None,
        "--config",
        exists=True,
        show_default=False,
        help="[Optional] Path to the configuration file",
    ),
    visualize: bool = typer.Option(
        False,
        "--visualize",
        "-v",
        help="[Optional] Open an online visualization of the mapping system",
        rich_help_panel="Additional Options",
    ),
):
    # Argument parsing
    visual_odometry = not just_mapping
    dataset_has_poses = True
    if visual_odometry:
        dataset_has_poses = False
    if not dataloader:
        dataloader = "generic"
    config = load_config(config_filename)

    # Initialization
    dataset = dataset_factory(
        dataloader=dataloader,
        dataset_folder=dataset_folder,
        has_poses=dataset_has_poses,
        depth_scale=config.dataset.depth_scale,
    )

    # Run pipeline
    MappingPipeline(
        dataset=dataset,
        config=config,
        visual_odometry=visual_odometry,
        visualize=visualize,
    ).run()


def run():
    app()
