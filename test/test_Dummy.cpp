#include <boost/test/unit_test.hpp>
#include <projection/Dummy.hpp>

using namespace projection;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    projection::DummyClass dummy;
    dummy.welcome();
}
