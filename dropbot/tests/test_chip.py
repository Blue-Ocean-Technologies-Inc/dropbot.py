from dropbot.chip import get_all_intersections, draw, get_channel_neighbours
import svg_model

SVG_PATH = 'C:\\Users\\vr372\OneDrive - Yale University\Documents\py_projects\dropbot.py\dropbot\static\SCI-BOTS 90-pin array\device.svg'


def test_intersections_from_svg():
    df_intersections = get_all_intersections(SVG_PATH)

    # Test device has 92 electrodes (each electrode has a unique id)
    # connected to 90 channels (two pairs of electrodes are each attached to a common electrode).
    unique_ids = df_intersections.index.get_level_values('id').drop_duplicates()
    assert unique_ids.shape[0] == 92


def test_intersections_from_frame():
    # Load data frame containing vertices from example SVG fil
    df_shapes = svg_model.svg_shapes_to_df(SVG_PATH)
    df_intersections = get_all_intersections(df_shapes)

    # Test device has 92 electrodes (each electrode has a unique id)
    # connected to 90 channels (two pairs of electrodes are each attached to a common electrode).
    unique_ids = df_intersections.index.get_level_values('id').drop_duplicates()
    assert unique_ids.shape[0] == 92


def test_draw_from_svg():
    # Draw detected neighbours.
    out_dict = draw(SVG_PATH)
    assert list(out_dict.keys()) == ['axis', 'electrode_channels', 'df_shapes', 'channel_patches']


def test_get_channel_neighbours():
    channel_neighbours = get_channel_neighbours(SVG_PATH)

    # Each channel should have 4 neighbours: `up`, `down`, `left`, and `right`.
    # If one of the neighbours does not exist, the corresponding entry is set to `NaN`.
    counts = channel_neighbours.fillna(-1).groupby(level='channel').count()
    assert counts.min().min() == 4  # Check minimum count across all channels
    assert counts.max().max() == 4  # Check maximum count across all channels


if __name__ == '__main__':
    import pytest

    pytest.main([__file__])
