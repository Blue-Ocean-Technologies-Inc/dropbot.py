import nose.tools

from dropbot.chip import get_all_intersections, draw, get_channel_neighbours
from dropbot import DATA_DIR
import svg_model


SVG_PATH = DATA_DIR.joinpath('SCI-BOTS 90-pin array', 'device.svg')


def test_intersections_from_svg():
    df_intersections = get_all_intersections(SVG_PATH)

    # Test device has 92 electrodes (each electrode has a unique id). connected
    # to 90 channels (two pairs of electrodes are each attached to a common
    # electrode).
    nose.tools.eq_(92, df_intersections.index.get_level_values('id')
                   .drop_duplicates().shape[0])


def test_intersections_from_frame():
    # Load data frame containing vertices from example SVG file.
    df_shapes = svg_model.svg_shapes_to_df(SVG_PATH)
    df_intersections = get_all_intersections(df_shapes)

    # Test device has 92 electrodes (each electrode has a unique id). connected
    # to 90 channels (two pairs of electrodes are each attached to a common
    # electrode).
    nose.tools.eq_(92, df_intersections.index.get_level_values('id')
                   .drop_duplicates().shape[0])


def test_draw_from_svg():
    # Draw detected neighbours.
    ax, df_intersections = draw(SVG_PATH)


def test_get_channel_neighbours():
    channel_neighbours = get_channel_neighbours(SVG_PATH)

    # Each channel should have 4 neighbours: `up`, `down`, `left`, and `right`.
    # In the case where one of the neighbours does not exist, the corresponding
    # entry is set `NaN` (which can be replaced using the `fillna` method).
    counts = channel_neighbours.fillna(-1).groupby(level='channel').count()
    nose.tools.eq_(4, counts.min())
    nose.tools.eq_(4, counts.max())
