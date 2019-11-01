
SECTIONS
{
  /* device has small L2, so put this in MSMC to make it fit */
  .far:timestamps > MSMCSRAM
}
