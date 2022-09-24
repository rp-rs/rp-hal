use super::{single_channel::ChannelPair, ReadTarget, WriteTarget};
#[allow(dead_code)]
pub struct RingTransfer<CH: ChannelPair, FROM: ReadTarget, TO: WriteTarget> {
    ch: CH,
    from: FROM,
    to: TO,
}

impl<CH, FROM, TO> RingTransfer<CH, FROM, TO>
where
    CH: ChannelPair,
    FROM: ReadTarget,
    TO: WriteTarget,
{
    // TODO
}
